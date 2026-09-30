#!/usr/bin/env python3
"""Check a real ``fod-vision`` capture against nav's parser, and record what nav makes of it.

``fodnav-replay`` answers "what would nav have seen". This answers the question
before that one: **does nav read his output correctly at all?** It is written
for captures from Bthcorn's ``scripts/capture_raw.py``, which writes one
``detail()`` dict per line plus two fields of its own:

* ``t`` -- wall-clock seconds at capture, and
* ``targets`` -- **his** ``latest()`` for the same frame.

That second field is the useful one. ``parse_detail`` does not read it: it
rebuilds ``latest()`` from ``tracks`` by dropping coasting tracks, because a
plain ``detail()`` log has no ``targets``. Here both are on the same line, so
the reconstruction can be checked against his answer frame by frame.

What it checks, per capture:

1. every line is JSON and every record/track has the keys the vendored spec
   (``docs/vendor/fod-vision-v0.3.0-INTEGRATION.md``) lists;
2. enum values, box sanity, and that ``centroid`` is the box centre;
3. nav's ``latest()`` equals his ``targets`` -- ids, boxes, states;
4. frame ids, timing, and the things nav is told never to rely on (``cls``
   flipping on one track id);
5. one object held as two live tracks (box IoU > ``DUPLICATE_IOU``), and how
   many of nav's target switches are only between those two copies.

Layout. One directory per capture session, raw input under ``data/``, and
this tool's output mirrored under ``output/``::

    cv_tests/2026-09-24/
      data/<capture>/        meta.json + *.jsonl, exactly as he handed them over
      output/summary.md      the cross-capture write-up (by hand, not generated)
      output/<capture>/
        summary.md           generated: the verdict, an input -> output section
                             (the step-by-step funnel, then worked frames: every
                             track he sent, what nav did with it, and why), the
                             numbers, and how this capture compares with the others
        audit.json           the same numbers, machine-readable
        nav_frames.jsonl     one line per input frame: what nav parsed, what it
                             would chase, where the projection puts it, and
                             which one it would pick
        replay.txt           ``fodnav-replay`` on the same log, verbatim

**The metres are not measurements.** No ground calibration exists for the
camera these were taken with -- its mount is PRD O-3, unmeasured. Projection
uses ``config/sim_ground_homography.json``, a fictional 18 cm / 25 deg mount at
the same 1280x720, so it exercises the code path and the range gates and says
nothing about where the objects were on the floor.

    uv run python tools/audit_cv_capture.py cv_tests/2026-09-24
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fodnav.config import load_nav_config, load_robot_config  # noqa: E402
from fodnav.frames import Pose2D  # noqa: E402
from fodnav.ground import GroundProjector, load_ground_calibration  # noqa: E402
from fodnav.link.vision import CONFIRM, PICK, parse_detail, select_targets  # noqa: E402
from fodnav.target import TargetParams, TargetSet  # noqa: E402
from fodnav.cli import replay as replay_cli  # noqa: E402

#: From the vendored spec, ``vision.detail()`` row.
DETAIL_KEYS = {"frame_id", "age", "blocked", "fps", "stage_ms", "top_scores",
               "camera", "tracks", "error"}
TRACK_KEYS = {"id", "state", "action", "cls", "conf", "raw", "hits", "misses",
              "box", "centroid", "in_zone"}
TARGET_KEYS = {"id", "state", "action", "cls", "conf", "box", "centroid"}
CAMERA_KEYS = {"zoom", "rotate", "conf", "frame_size", "imgsz", "focus_m"}
STAGE_KEYS = {"capture", "preprocess", "infer", "postprocess", "total"}
STATES = {"CONFIRM", "CAUTION", "IGNORE"}
ACTIONS = {"PICK", "REPORT", "IGNORE"}


def _pct(xs: list[float], q: float) -> float | None:
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))]


def _r(x: float | None, n: int = 3) -> float | None:
    return None if x is None else round(x, n)


def _iou(a: list[int], b: list[int]) -> float:
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union else 0.0


#: Two live tracks whose boxes overlap this much are one object tracked twice.
DUPLICATE_IOU = 0.7


def _nonfinite(obj: Any, path: str = "") -> list[str]:
    """Paths of every inf/nan in a record. The spec says ``age`` and ``focus_m`` may be."""
    out: list[str] = []
    if isinstance(obj, float) and not math.isfinite(obj):
        out.append(path)
    elif isinstance(obj, dict):
        for k, v in obj.items():
            out += _nonfinite(v, f"{path}.{k}" if path else k)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            out += _nonfinite(v, f"{path}[{i}]")
    return out


def audit(capture_dir: Path, out_dir: Path, robot_cfg: str, calib_path: str) -> dict[str, Any]:
    jsonl = sorted(capture_dir.glob("*.jsonl"))
    if not jsonl:
        raise SystemExit(f"{capture_dir}: no .jsonl files")
    meta = json.loads((capture_dir / "meta.json").read_text()) if (capture_dir / "meta.json").exists() else {}

    robot = load_robot_config(robot_cfg)
    nav = load_nav_config()
    calib = load_ground_calibration(calib_path, robot)
    projector = GroundProjector(calib, max_valid_range_m=robot.get("camera.fov_far_limit_m"))
    tracker = TargetSet(TargetParams.from_config(nav))

    problems: Counter[str] = Counter()
    examples: dict[str, str] = {}

    def problem(kind: str, where: str) -> None:
        problems[kind] += 1
        examples.setdefault(kind, where)

    n_lines = n_bad_json = 0
    extra_keys: Counter[str] = Counter()
    nonfinite: Counter[str] = Counter()
    frame_ids: list[int] = []
    ts: list[float] = []
    infer_ms: list[float] = []
    total_ms: list[float] = []
    frame_sizes: Counter[str] = Counter()
    states: Counter[str] = Counter()
    actions: Counter[str] = Counter()
    n_tracks = n_coasting = n_latest = n_his_targets = 0
    n_match = n_mismatch = n_no_targets_field = 0
    n_blocked = n_error = 0
    cls_by_id: dict[int, set[str]] = defaultdict(set)
    state_seq: dict[int, list[str]] = defaultdict(list)
    transitions: Counter[str] = Counter()
    ground_minus_centroid_px: list[float] = []
    huge_confirmed: list[dict[str, Any]] = []
    n_chased = n_projected = n_rejected = 0
    chased_ids: set[int] = set()
    ranges: list[float] = []
    best_ids: Counter[int] = Counter()
    n_frames_with_best = 0
    pool: list[dict[str, Any]] = []  # per-frame input/output, for the worked examples
    n_frames_dup = n_frames_dup_confirm = 0
    dup_pairs: set[frozenset[int]] = set()
    dup_cls_differs = dup_cls_same = 0
    prev_best: int | None = None
    n_switches = n_switches_dup = 0

    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "nav_frames.jsonl").open("w", encoding="utf-8") as nav_out:
        for path in jsonl:
            for lineno, line in enumerate(path.open(encoding="utf-8"), 1):
                line = line.strip()
                if not line:
                    continue
                n_lines += 1
                where = f"{path.name}:{lineno}"
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    n_bad_json += 1
                    problem("bad_json", where)
                    continue

                # -- 1. schema ------------------------------------------------
                for k in DETAIL_KEYS - d.keys():
                    problem(f"missing_key:{k}", where)
                for k in d.keys() - DETAIL_KEYS:
                    extra_keys[k] += 1
                cam = d.get("camera") or {}
                for k in CAMERA_KEYS - cam.keys():
                    problem(f"missing_camera_key:{k}", where)
                for k in STAGE_KEYS - (d.get("stage_ms") or {}).keys():
                    problem(f"missing_stage_key:{k}", where)
                for p in _nonfinite(d):
                    nonfinite[p.split("[")[0]] += 1

                fs = tuple(cam.get("frame_size") or (0, 0))
                frame_sizes[f"{fs[0]}x{fs[1]}"] += 1
                if "frame_size" in meta and list(fs) != list(meta["frame_size"]):
                    problem("frame_size_differs_from_meta", where)
                if isinstance(d.get("frame_id"), int):
                    frame_ids.append(d["frame_id"])
                if isinstance(d.get("t"), (int, float)):
                    ts.append(float(d["t"]))
                sm = d.get("stage_ms") or {}
                if isinstance(sm.get("infer"), (int, float)):
                    infer_ms.append(sm["infer"])
                if isinstance(sm.get("total"), (int, float)):
                    total_ms.append(sm["total"])
                n_blocked += bool(d.get("blocked"))
                n_error += d.get("error") is not None

                # -- 2. per-track sanity -------------------------------------
                for tr in d.get("tracks") or []:
                    n_tracks += 1
                    for k in TRACK_KEYS - tr.keys():
                        problem(f"missing_track_key:{k}", where)
                    st, ac = tr.get("state"), tr.get("action")
                    states[st] += 1
                    actions[ac] += 1
                    if st not in STATES:
                        problem(f"unknown_state:{st}", where)
                    if ac not in ACTIONS:
                        problem(f"unknown_action:{ac}", where)
                    if tr.get("misses", 0):
                        n_coasting += 1
                    box = tr.get("box") or [0, 0, 0, 0]
                    x0, y0, x1, y1 = box
                    if not all(isinstance(v, int) for v in box):
                        problem("box_not_int", where)
                    if not (x0 < x1 and y0 < y1):
                        problem("box_degenerate", where)
                    if fs[0] and not (0 <= x0 and x1 <= fs[0] and 0 <= y0 and y1 <= fs[1]):
                        problem("box_outside_frame", where)
                    c = tr.get("centroid") or [None, None]
                    if c[0] is None or abs(c[0] - (x0 + x1) / 2) > 0.5 or abs(c[1] - (y0 + y1) / 2) > 0.5:
                        problem("centroid_not_box_centre", where)
                    if fs[0] and st == CONFIRM and (x1 - x0) > 0.8 * fs[0]:
                        huge_confirmed.append({"frame_id": d.get("frame_id"), "id": tr.get("id"),
                                               "conf": _r(tr.get("conf")), "box": box})
                    if tr.get("misses", 0) == 0:
                        ground_minus_centroid_px.append(y1 - (y0 + y1) / 2)
                        cls_by_id[tr["id"]].add(tr.get("cls"))
                        seq = state_seq[tr["id"]]
                        if seq and seq[-1] != st:
                            transitions[f"{seq[-1]}->{st}"] += 1
                        seq.append(st)

                # -- 2b. one object, two tracks ------------------------------
                live = [tr for tr in d.get("tracks") or [] if not tr.get("misses", 0)]
                any_dup = any_dup_confirm = False
                for i, a in enumerate(live):
                    for b in live[i + 1:]:
                        if _iou(a["box"], b["box"]) > DUPLICATE_IOU:
                            any_dup = True
                            dup_pairs.add(frozenset((a["id"], b["id"])))
                            if a.get("cls") == b.get("cls"):
                                dup_cls_same += 1
                            else:
                                dup_cls_differs += 1
                            any_dup_confirm |= a["state"] == b["state"] == CONFIRM
                n_frames_dup += any_dup
                n_frames_dup_confirm += any_dup_confirm

                # -- 3. nav's parse vs his latest() --------------------------
                frame = parse_detail(d, t_recv=0.0)
                ours = sorted((t.id, t.box, t.state) for t in frame.targets)
                n_latest += len(ours)
                if "targets" in d:
                    theirs = sorted((int(t["id"]), tuple(int(v) for v in t["box"]), t["state"])
                                    for t in d["targets"])
                    n_his_targets += len(theirs)
                    for t in d["targets"]:
                        for k in TARGET_KEYS - t.keys():
                            problem(f"missing_target_key:{k}", where)
                    if ours == theirs:
                        n_match += 1
                    else:
                        n_mismatch += 1
                        problem("latest_mismatch", where)
                else:
                    n_no_targets_field += 1
                    theirs = None

                # -- 4. what nav does with it --------------------------------
                chased = []
                for t in select_targets(frame, states=(CONFIRM,), actions=(PICK,)):
                    n_chased += 1
                    chased_ids.add(t.id)
                    p = projector.project_pixel(*t.ground_px)
                    rec: dict[str, Any] = {"id": t.id, "conf": _r(t.conf), "cls": t.cls,
                                           "box": list(t.box), "ground_px": list(t.ground_px)}
                    if p is None:
                        n_rejected += 1
                        rec["projected"] = None
                    else:
                        n_projected += 1
                        ranges.append(math.hypot(p.x, p.y))
                        rec["projected"] = {"x_m": _r(p.x), "y_m": _r(p.y),
                                            "range_m": _r(math.hypot(p.x, p.y)),
                                            "bearing_deg": _r(math.degrees(math.atan2(p.y, p.x)), 1)}
                    chased.append(rec)

                now = d.get("t", 0.0)
                if frame.frame_size == projector.frame_size and frame.ok:
                    tracker.update(frame, projector, Pose2D(), now)
                best = tracker.best(Pose2D()) if frame.ok else None
                if best is not None:
                    n_frames_with_best += 1
                    best_ids[best.id] += 1
                    if prev_best is not None and best.id != prev_best:
                        n_switches += 1
                        n_switches_dup += frozenset((best.id, prev_best)) in dup_pairs
                    prev_best = best.id

                chased_ids_here = {c["id"] for c in chased}
                held = {}
                for tid, trk in tracker.tracks.items():
                    if tid in chased_ids_here:
                        continue
                    g = trk.predict_base(Pose2D())
                    held[tid] = {"age_s": _r(trk.age_s(now)), "x_m": _r(g.x), "y_m": _r(g.y),
                                 "range_m": _r(g.range_m),
                                 "bearing_deg": _r(math.degrees(math.atan2(g.y, g.x)), 1)}
                pool.append({
                    "source": f"{path.name}:{lineno}",
                    "frame_id": frame.frame_id,
                    "frame_size": list(fs),
                    "blocked": frame.blocked,
                    "tracks": [{k: tr.get(k) for k in ("id", "state", "action", "cls", "conf",
                                                        "misses", "box", "centroid")}
                               for tr in d.get("tracks") or []],
                    "chased": chased,
                    "held": held,
                    "best": None if best is None else best.id,
                })

                nav_out.write(json.dumps({
                    "frame_id": frame.frame_id,
                    "t": d.get("t"),
                    "error": frame.error,
                    "blocked": frame.blocked,
                    "n_tracks": len(d.get("tracks") or []),
                    "latest_ids": [i for i, _, _ in ours],
                    "his_target_ids": None if theirs is None else [i for i, _, _ in theirs],
                    "latest_matches_his": None if theirs is None else ours == theirs,
                    "chased": chased,
                    "best": None if best is None else {
                        "id": best.id, "range_m": _r(best.range_m),
                        "bearing_deg": _r(math.degrees(best.bearing_rad), 1),
                        "age_s": _r(best.age_s(now)),
                    },
                }, separators=(",", ":")) + "\n")

    gaps = [b - a for a, b in zip(frame_ids, frame_ids[1:])]
    dts = [b - a for a, b in zip(ts, ts[1:])]
    flips = {i: sorted(c) for i, c in cls_by_id.items() if len(c) > 1}
    duration = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0

    summary: dict[str, Any] = {
        "capture": str(capture_dir),
        "hef": meta.get("hef"),
        "imgsz": (meta.get("camera") or {}).get("imgsz"),
        "host_conf": (meta.get("camera") or {}).get("conf"),
        "parse": {
            "lines": n_lines,
            "bad_json": n_bad_json,
            "frames_parsed": n_lines - n_bad_json,
            "problems": dict(problems),
            "first_example": examples,
            "extra_top_level_keys": dict(extra_keys),
            "non_finite_values": dict(nonfinite),
            "frame_sizes": dict(frame_sizes),
        },
        "latest_reconstruction": {
            "frames_with_his_targets": n_match + n_mismatch,
            "frames_without": n_no_targets_field,
            "match": n_match,
            "mismatch": n_mismatch,
            "targets_ours": n_latest,
            "targets_his": n_his_targets,
        },
        "stream": {
            "duration_s": _r(duration, 2),
            "frame_id_first_last": [frame_ids[0], frame_ids[-1]] if frame_ids else None,
            "frame_id_non_monotonic": sum(1 for g in gaps if g <= 0),
            "frames_skipped_in_log": sum(g - 1 for g in gaps if g > 1),
            "effective_fps": _r((len(ts) - 1) / duration, 1) if duration else None,
            "dt_ms_median_p95": [_r(1e3 * _pct(dts, 0.5), 1), _r(1e3 * _pct(dts, 0.95), 1)] if dts else None,
            "infer_ms_median_p95": [_r(_pct(infer_ms, 0.5), 1), _r(_pct(infer_ms, 0.95), 1)],
            "total_ms_median_p95": [_r(_pct(total_ms, 0.5), 1), _r(_pct(total_ms, 0.95), 1)],
            "error_frames": n_error,
            "blocked_frames": n_blocked,
        },
        "tracks": {
            "track_rows": n_tracks,
            "coasting_rows": n_coasting,
            "distinct_ids": len(cls_by_id),
            "by_state": dict(states),
            "by_action": dict(actions),
            "state_transitions": dict(transitions),
            "ids_whose_cls_flips": len(flips),
            "cls_flip_examples": {str(k): v for k, v in list(flips.items())[:5]},
            "confirmed_boxes_over_80pct_frame_width": huge_confirmed[:10],
            "duplicates": {
                "iou_threshold": DUPLICATE_IOU,
                "frames_with_a_duplicate": n_frames_dup,
                "frames_with_a_confirmed_duplicate": n_frames_dup_confirm,
                "distinct_id_pairs": len(dup_pairs),
                "pairs_cls_differs": dup_cls_differs,
                "pairs_cls_same": dup_cls_same,
            },
            "ground_below_centroid_px_median_max": [
                _r(statistics.median(ground_minus_centroid_px), 1),
                _r(max(ground_minus_centroid_px), 1),
            ] if ground_minus_centroid_px else None,
        },
        "nav": {
            "calibration": calib_path,
            "calibration_is_fictional": True,
            "chased_target_rows": n_chased,
            "distinct_chased_ids": len(chased_ids),
            "projected": n_projected,
            "rejected_by_projector": n_rejected,
            "projector_stats": projector.stats(),
            "range_m_min_median_max": [_r(min(ranges)), _r(statistics.median(ranges)), _r(max(ranges))] if ranges else None,
            "frames_with_a_best_target": n_frames_with_best,
            "best_target_switches": n_switches,
            "best_target_switches_between_duplicates": n_switches_dup,
            "best_target_ids": dict(best_ids.most_common(5)),
        },
    }
    summary["examples"] = pick_examples(pool, huge_confirmed, tracker.params.max_age_s)
    (out_dir / "audit.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def _verdicts(ex: dict[str, Any]) -> list[dict[str, Any]]:
    """One row per input track, plus one per track nav holds that is not in the input.

    Each row says what nav did with it and why, in terms of the rule that decided it.
    """
    chased = {c["id"]: c for c in ex["chased"]}
    held = ex["held"]
    best = ex["best"]
    candidates = [c for c in ex["chased"] if c["projected"]] + [{"id": i, **h} for i, h in held.items()]
    n_cand = len(candidates)

    def rng(i: int) -> float | None:
        if i in chased and chased[i]["projected"]:
            return chased[i]["projected"]["range_m"]
        return held[i]["range_m"] if i in held else None

    rows = []
    seen = set()
    for tr in ex["tracks"]:
        i = tr["id"]
        seen.add(i)
        x0, y0, x1, y1 = tr["box"]
        row: dict[str, Any] = {"input": tr, "ground_px": None, "floor": None}
        width = (x1 - x0) / ex["frame_size"][0] if ex["frame_size"][0] else 0
        if tr["misses"]:
            row["verdict"] = "dropped (coasting)"
            row["why"] = (f"His tracker did not see it this frame (`misses` {tr['misses']}). His "
                          "`latest()` leaves coasting tracks out, and so does nav.")
        elif tr["state"] != CONFIRM:
            row["verdict"] = f"ignored ({tr['state']})"
            row["why"] = (f"In `latest()`, but his tracker has not confirmed it (conf {tr['conf']:.2f}; "
                          "CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own.")
        elif tr["action"] != PICK:
            row["verdict"] = f"ignored ({tr['action']})"
            row["why"] = f"`action` is {tr['action']}; nav chases PICK only."
        else:
            c = chased[i]
            row["ground_px"] = c["ground_px"]
            dy = y1 - (y0 + y1) / 2
            if c["projected"] is None:
                row["verdict"] = "rejected by projector"
                row["why"] = ("CONFIRM + PICK, but the ground point is above the horizon or past "
                              "`fov_far_limit_m`, so there is no floor position to drive to.")
            else:
                row["floor"] = c["projected"]
                if i == best:
                    row["verdict"] = "**chosen**"
                    row["why"] = (f"CONFIRM + PICK, and the nearest of {n_cand} candidate(s), so nav "
                                  f"drives at it. Projected from the bottom-centre, {dy:.0f} px below "
                                  "the centroid.")
                else:
                    row["verdict"] = "candidate"
                    row["why"] = (f"CONFIRM + PICK, projected from the bottom-centre. Not chosen: "
                                  f"id {best} is nearer ({rng(best)} m against {c['projected']['range_m']} m"
                                  + (", a tie at the millimetre" if rng(best) == c["projected"]["range_m"] else "")
                                  + ").")
            for o in ex["tracks"]:
                if o["id"] != i and not o["misses"] and _iou(tr["box"], o["box"]) > DUPLICATE_IOU:
                    row["why"] += (f" **Same object as id {o['id']}** (IoU "
                                   f"{_iou(tr['box'], o['box']):.2f}, `{tr['cls']}` vs `{o['cls']}`): "
                                   "his tracker holds one object as two tracks.")
            if tr["conf"] < 0.5:
                row["why"] += (f" Conf {tr['conf']:.2f} is under 0.5, but his latch holds CONFIRM "
                               "until 0.25.")
            if width > 0.8:
                row["why"] += (f" **The box spans {width:.0%} of the frame width** -- not a fastener, "
                               "but nav has no size gate.")
        if i in held and not row["floor"]:
            h = held[i]
            row["floor"] = h
            row["verdict"] = "**chosen** (held)" if i == best else row["verdict"] + ", still held"
            row["why"] += (f" Nav still holds its last floor fix, {h['age_s']} s old -- inside "
                           "`detections.max_age_s`"
                           + (", and it is still the nearest, so nav keeps driving at it." if i == best else "."))
        rows.append(row)
    for i, h in held.items():
        if i in seen:
            continue
        rows.append({"input": None, "id": i, "ground_px": None, "floor": h,
                     "verdict": "**chosen** (held)" if i == best else "held",
                     "why": (f"Gone from his list, but last seen as CONFIRM {h['age_s']} s ago, inside "
                             "`detections.max_age_s`, so it is still a candidate"
                             + (" -- and the nearest." if i == best else "."))})
    return rows


def pick_examples(pool: list[dict[str, Any]], huge: list[dict[str, Any]],
                  max_age_s: float) -> list[dict[str, Any]]:
    """Pick frames worth reading row by row: the most instructive typical one, plus any whole-frame box."""
    def kinds(ex: dict[str, Any]) -> set[str]:
        return {r["verdict"].split(" (")[0].split(",")[0].strip("*") for r in _verdicts(ex)}

    out = []
    usable = [ex for ex in pool if ex["best"] is not None]
    small = [ex for ex in usable if len(ex["tracks"]) + len(ex["held"]) <= 6] or usable
    if small:
        typical = max(small, key=lambda ex: (len(kinds(ex)), -len(ex["tracks"]), -ex["frame_id"]))
        out.append({"title": "a typical frame, with as many kinds of row as possible",
                    **typical, "rows": _verdicts(typical)})
    if huge:
        fid = huge[0]["frame_id"]
        ex = next((e for e in pool if e["frame_id"] == fid), None)
        if ex is not None:
            out.append({"title": "the whole-frame box that reaches CONFIRM", **ex, "rows": _verdicts(ex)})
    return out


def render_io(s: dict[str, Any], name: str) -> list[str]:
    """The input -> output section: the whole-capture funnel, then worked frames."""
    p, lr, tr, nv = s["parse"], s["latest_reconstruction"], s["tracks"], s["nav"]
    frames = p["frames_parsed"]
    live = tr["track_rows"] - tr["coasting_rows"]
    lines = [
        "## Input → output",
        "",
        "What goes into nav from his log, what comes out, and the rule that decides each step. "
        "Metres use the **fictional** sim calibration; they show the code path, not where anything was.",
        "",
        "### The whole capture",
        "",
        "| Step | In (his data) | Out (nav) | Why |",
        "|---|---|---|---|",
        f"| Read the log | {p['lines']} lines | {frames} frames | Each line is one `detail()`; "
        "nav's parser, `parse_detail`, reads it directly. |",
        f"| Drop coasting tracks | {tr['track_rows']} track rows | {live} rows "
        f"(his `targets`: {lr['targets_his']}) | A coasting track (`misses` > 0) was not seen this "
        "frame. His `latest()` drops them; nav rebuilds the same list from `tracks`. |",
        f"| Keep CONFIRM + PICK | {live} rows | {nv['chased_target_rows']} rows, "
        f"{nv['distinct_chased_ids']} ids | His tracker decides what is real (CONFIRM at 0.5, "
        "latched to 0.25). Nav adds no hysteresis of its own. |",
        f"| Project to the floor | {nv['chased_target_rows']} boxes | {nv['projected']} floor points, "
        f"{nv['rejected_by_projector']} rejected | Bottom-centre of the box, not the centroid: the "
        "bottom edge is where the object touches the floor. |",
        f"| Pick one | {frames} frames | {nv['frames_with_a_best_target']} frames with a target | "
        "Nearest floor point, including tracks last seen ≤ `max_age_s` ago. Nearest is cheapest "
        "to reach. |",
        "",
    ]
    for ex in s.get("examples", []):
        best = next((r for r in ex["rows"] if r["verdict"].startswith("**chosen")), None)
        drive = ("nothing to drive at" if best is None else
                 f"drive at id **{ex['best']}**, {best['floor']['range_m']} m at "
                 f"{best['floor']['bearing_deg']:+}°")
        lines += [
            f"### Frame {ex['frame_id']} — {ex['title']}",
            "",
            f"**In:** `../../data/{name}/{ex['source']}`, {len(ex['tracks'])} tracks, "
            f"`frame_size` {ex['frame_size'][0]}×{ex['frame_size'][1]}.  ",
            f"**Out:** {drive}.",
            "",
            "| id | state | misses | conf | box (px) | → | nav | ground pt (px) | floor x, y (m) | range, bearing | why |",
            "|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for r in ex["rows"]:
            t = r["input"]
            if t is None:
                inp = f"{r['id']} | — | — | — | *not in this frame* "
            else:
                inp = (f"{t['id']} | {t['state']} | {t['misses']} | {t['conf']:.2f} | "
                       f"`{t['box']}` ")
            g = "—" if r["ground_px"] is None else f"({r['ground_px'][0]:.0f}, {r['ground_px'][1]:.0f})"
            f = r["floor"]
            fl = "—" if f is None else f"{f['x_m']:.3f}, {f['y_m']:+.3f}"
            rb = "—" if f is None else f"{f['range_m']:.3f} m, {f['bearing_deg']:+.1f}°"
            lines.append(f"| {inp}| → | {r['verdict']} | {g} | {fl} | {rb} | {r['why']} |")
        lines.append("")
    return lines


def render_summary(name: str, s: dict[str, Any], session: dict[str, dict[str, Any]]) -> str:
    """One capture's ``summary.md``: the verdict first, then the numbers beside its siblings."""
    p, lr, st, tr, nv = s["parse"], s["latest_reconstruction"], s["stream"], s["tracks"], s["nav"]
    clean = not p["problems"] and lr["mismatch"] == 0
    names = list(session)

    def row(label: str, f) -> str:
        cells = []
        for n in names:
            v = f(session[n])
            cells.append(f"**{v}**" if n == name else str(v))
        return f"| {label} | " + " | ".join(cells) + " |"

    def pct(a: int, b: int) -> str:
        return f"{a}/{b}"

    def mp(x):  # median / p95 pair
        return "–" if not x else f"{x[0]} / {x[1]}"

    lines = [
        f"# {name} — what nav made of it",
        "",
        f"Input: `../../data/{name}/`. Model `{Path(s['hef'] or '?').parent.parent.name}/"
        f"{Path(s['hef'] or '?').parent.name}`, imgsz {s['imgsz']}, host conf {s['host_conf']}.",
        "",
        "## Verdict",
        "",
        ("**Parsable.** " if clean else "**NOT cleanly parsable.** ")
        + f"{p['frames_parsed']}/{p['lines']} lines parsed, "
        + f"{len(p['problems'])} kinds of schema problem, and nav's `latest()` matched "
        + f"his `targets` on {lr['match']}/{lr['match'] + lr['mismatch']} frames "
        + f"({lr['targets_ours']} targets ours, {lr['targets_his']} his).",
        "",
    ]
    if p["problems"]:
        lines += ["Problems (first example of each):", ""]
        lines += [f"- `{k}` × {v} — {p['first_example'].get(k, '')}" for k, v in p["problems"].items()]
        lines.append("")
    lines += [
    ]
    lines += render_io(s, name)
    lines += [
        "## What nav would do",
        "",
        f"- Chase mode (CONFIRM + PICK) had a target in **{nv['frames_with_a_best_target']}"
        f"/{p['frames_parsed']}** frames, from {nv['distinct_chased_ids']} of his "
        f"{tr['distinct_ids']} track ids.",
        f"- Projector rejected {nv['rejected_by_projector']} of {nv['chased_target_rows']} chased targets.",
        f"- Most-chosen targets (id: frames): "
        + ", ".join(f"{k}: {v}" for k, v in nv["best_target_ids"].items()) + ".",
        f"- Range {' / '.join(map(str, nv['range_m_min_median_max'] or []))} m (min / median / max) — "
        "**fictional calibration, not a measurement.**",
        "",
        "## Worth knowing",
        "",
        f"- Logged at **{st['effective_fps']} FPS**; his thread completed frames "
        f"{st['frame_id_first_last'][0]}→{st['frame_id_first_last'][1]} "
        f"and {st['frames_skipped_in_log']} are not in the log. Infer {mp(st['infer_ms_median_p95'])} ms "
        "(median / p95).",
        f"- `cls` flips on {tr['ids_whose_cls_flips']}/{tr['distinct_ids']} track ids — "
        "never branch on it.",
        "- State transitions seen: "
        + (", ".join(f"{k.replace('->', ' → ')} ×{v}" for k, v in tr["state_transitions"].items()) or "none")
        + ".",
    ]
    if tr["confirmed_boxes_over_80pct_frame_width"]:
        ex = tr["confirmed_boxes_over_80pct_frame_width"][0]
        lines.append(
            f"- **{len(tr['confirmed_boxes_over_80pct_frame_width'])} CONFIRM box(es) wider than 80% of "
            f"the frame**, first at frame {ex['frame_id']} (id {ex['id']}, conf {ex['conf']}, "
            f"box {ex['box']}). Nav has no size gate and will chase it."
        )
    lines += [
        f"- **One object, two tracks:** a pair of live tracks overlap at IoU > {tr['duplicates']['iou_threshold']} "
        f"in {tr['duplicates']['frames_with_a_duplicate']}/{p['frames_parsed']} frames "
        f"({tr['duplicates']['frames_with_a_confirmed_duplicate']} with both CONFIRM); "
        f"{tr['duplicates']['pairs_cls_differs']} of "
        f"{tr['duplicates']['pairs_cls_differs'] + tr['duplicates']['pairs_cls_same']} such pairs carry "
        f"different `cls`. Of nav's {nv['best_target_switches']} target switches, "
        f"**{nv['best_target_switches_between_duplicates']} are between two tracks of the same object.**",
        f"- `blocked` true on {st['blocked_frames']}/{p['frames_parsed']} frames (nav ignores it in chase mode).",
        "- Extra top-level keys not in the `detail()` spec: "
        + (", ".join(f"`{k}`" for k in sorted(p["extra_top_level_keys"])) or "none") + " "
        "(added by `capture_raw.py`; nav ignores them).",
        "",
        "## Against the other captures in this session",
        "",
        "| | " + " | ".join(names) + " |",
        "|---|" + "---|" * len(names),
        row("Frames parsed", lambda x: pct(x["parse"]["frames_parsed"], x["parse"]["lines"])),
        row("`latest()` matches his", lambda x: pct(x["latest_reconstruction"]["match"],
                                                   x["parse"]["frames_parsed"])),
        row("Logged FPS", lambda x: x["stream"]["effective_fps"]),
        row("Infer ms, median / p95", lambda x: mp(x["stream"]["infer_ms_median_p95"])),
        row("Distinct track ids", lambda x: x["tracks"]["distinct_ids"]),
        row("…ever chased", lambda x: x["nav"]["distinct_chased_ids"]),
        row("Frames with a target", lambda x: pct(x["nav"]["frames_with_a_best_target"],
                                                  x["parse"]["frames_parsed"])),
        row("`cls` flips (ids)", lambda x: x["tracks"]["ids_whose_cls_flips"]),
        row("Frames with a same-object duplicate", lambda x: pct(
            x["tracks"]["duplicates"]["frames_with_a_duplicate"], x["parse"]["frames_parsed"])),
        row("Target switches, same-object / all", lambda x: pct(
            x["nav"]["best_target_switches_between_duplicates"], x["nav"]["best_target_switches"])),
        row("Whole-frame CONFIRM boxes", lambda x: len(x["tracks"]["confirmed_boxes_over_80pct_frame_width"])),
        "",
        "Files here: `audit.json` (these numbers), `nav_frames.jsonl` (nav's output per input "
        "frame), `replay.txt` (`fodnav-replay` verbatim). The cross-capture write-up is "
        "`../summary.md`.",
        "",
    ]
    return "\n".join(lines)


def run_replay(log: Path) -> str:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        replay_cli.main([str(log)])
    return buf.getvalue()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path,
                    help="a session directory holding data/<capture>/, e.g. cv_tests/2026-09-24")
    ap.add_argument("--robot-config", default="config/sim_robot.yaml")
    ap.add_argument("--ground-calib", default="config/sim_ground_homography.json")
    args = ap.parse_args(argv)

    captures = sorted(p for p in (args.session / "data").iterdir() if p.is_dir())
    if not captures:
        raise SystemExit(f"{args.session}/data: no capture directories")

    results: dict[str, dict[str, Any]] = {}
    ok = True
    for cap in captures:
        out = args.session / "output" / cap.name
        s = audit(cap, out, args.robot_config, args.ground_calib)
        (out / "replay.txt").write_text(
            "\n".join(run_replay(log) for log in sorted(cap.glob("*.jsonl"))))
        results[cap.name] = s
        lr, p = s["latest_reconstruction"], s["parse"]
        ok &= not p["problems"] and lr["mismatch"] == 0
        print(f"{cap.name}: {p['frames_parsed']}/{p['lines']} frames parsed, "
              f"{len(p['problems'])} problem kinds, latest() match "
              f"{lr['match']}/{lr['match'] + lr['mismatch']}  -> {out}")
    for name, s in results.items():
        (args.session / "output" / name / "summary.md").write_text(render_summary(name, s, results))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
