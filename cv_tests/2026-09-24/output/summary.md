# Real CV captures through nav — 2026-09-24

The first real detector output this repo has read. Three ~30 s captures from
Bthcorn's board (Pi 5 + Hailo-8, his `scripts/capture_raw.py`), one per model,
same scene, taken 14:13–14:15 on 2026-09-24. The raw captures are in `../data/`;
each one has its own `summary.md` beside this file, in `s640-c/`, `n640-c/` and
`n480-c/`.

**Short answer: nav parses all of it, and gets his `latest()` exactly right on
every frame.** 2130 of 2130 frames parse, zero schema problems, and nav's
reconstruction of `latest()` matches his own on 2130 of 2130 frames.

**What this does *not* show:** anything about metres. No ground calibration
exists for the camera these came from — the mount is PRD O-3 and unmeasured.
Projection below uses `config/sim_ground_homography.json`, a fictional
18 cm / 25° mount at the same 1280×720. The ranges prove the code path runs;
they are not where the objects were. **Do not quote them.**

Reproduce:

```bash
uv run python tools/audit_cv_capture.py cv_tests/2026-09-24
uv run fodnav-replay cv_tests/2026-09-24/data/n640-c/raw_part01.jsonl
uv run pytest tests/test_vision_real.py
```

## Key points

1. **The interface is proven.** Nav reads all 2130 frames with no field, box or
   `frame_size` disagreement, and nav's `latest()` matches his on every one. The
   integration risk that remains is the Pi install and the camera mount, not the format.
2. **His tracker often holds one object as two tracks.** In 31–43 % of frames two
   live tracks overlap at IoU > 0.7, and in 905 of 911 such pairs they carry
   *different* `cls` (e.g. `bolt` and `screw`), so this is most likely per-class
   NMS. It is harmless for steering, because both copies project to the same
   point on the floor. But **52 of nav's 79 target switches are between two
   copies of one object**, so a switch count or a per-id catch log overstates how
   much the robot changes its mind. The fix belongs on his side
   (class-agnostic NMS); nav should not deduplicate. (§4, finding 8)
3. **Nav will chase a near-whole-frame box** at start-up (`n640-c`, frame 4).
   There is no size gate. Raise it with Bthcorn before adding one. (Finding 3)
4. **Use an `n` model.** `s640` logs at 14 FPS; both `n` builds hold 30. (Finding 5)
5. **None of the metres mean anything yet.** They come from the fictional sim
   calibration until PRD O-3 is measured.

Each capture's `summary.md` has an **Input → output** section: the whole
capture as a table of steps (his rows in, nav's rows out, and the rule at each
step), then worked frames. Each worked frame lists every track his log sent,
what nav did with it, and why.

The tool regenerates everything in `output/<capture>/` — `summary.md`,
`audit.json`, `replay.txt`, and `nav_frames.jsonl`: one line per input frame
saying what nav parsed, which targets it would chase, where the projection puts
each, and which one it would pick. This file is written by hand and is not
regenerated.

---

## 1. The captures

| | `s640-c` | `n640-c` | `n480-c` |
|---|---|---|---|
| Model (`.hef`) | `arg-bolts-4-s-640` | `arg-bolts-4-n-640` (conf 1e-4 build) | `arg-bolts-4-n-640` at 480 |
| `imgsz` | 640 | 640 | 480 |
| Host `conf` filter | 0.25 | 0.25 | 0.25 |
| `frame_size` | 1280×720 | 1280×720 | 1280×720 |
| Records | 463 | 780 | 887 |
| Duration | 32.7 s | 26.2 s | 29.6 s |
| `frame_id` range | 1 → 653 | 2 → 788 | 3 → 892 |
| Frames not in log | 190 | 7 | 3 |
| Logged rate | **14.1 FPS** | 29.7 FPS | 29.9 FPS |
| Infer ms, median / p95 | **45.3 / 47.1** | 16.0 / 16.7 | 10.9 / 11.5 |
| Frame total ms, median / p95 | 50.0 / 52.1 | 33.2 / 38.1 | 33.2 / 39.0 |
| Error frames | 0 | 0 | 0 |
| `blocked` frames | 187 (40%) | 466 (60%) | 594 (67%) |

The `s` model misses the 33 ms frame, exactly as the vendored guide says it
would (50.2 ms there, 45–50 ms here). The two `n` builds hold 30 FPS.

## 2. Does nav parse it? — yes

| Check | `s640-c` | `n640-c` | `n480-c` |
|---|---|---|---|
| Lines that are valid JSON | 463/463 | 780/780 | 887/887 |
| Missing `detail()` / `camera` / `stage_ms` / track keys | 0 | 0 | 0 |
| Unknown `state` or `action` | 0 | 0 | 0 |
| Degenerate or out-of-frame boxes | 0 | 0 | 0 |
| `centroid` ≠ box centre | 0 | 0 | 0 |
| `frame_size` ≠ `meta.json` | 0 | 0 | 0 |
| Non-finite values (`Infinity`) | 0 | 0 | 0 |
| **Nav `latest()` == his `targets`, frames** | **463/463** | **780/780** | **887/887** |
| Targets, nav / his | 3040 / 3040 | 6471 / 6471 | 6965 / 6965 |
| Coasting track rows correctly excluded | 706 | 1608 | 1917 |

The `latest()` row is the strong one. A plain `detail()` has no `latest()` in
it, so `parse_detail` rebuilds it by dropping tracks with `misses > 0`. His
capture script happens to write his real `latest()` beside each `detail()` as
`targets`, so for the first time that reconstruction could be checked against
his answer rather than against our reading of his spec. It holds on every frame.

Two top-level keys are **not** in the vendored `detail()` spec: `t` (wall-clock
capture time) and `targets`. Both are added by `capture_raw.py`, not by
`Vision.detail()`. Nav ignores both, which is correct — a log nav writes itself
through `JsonlVisionLog` will not have them.

`fodnav-replay` reads all three logs whole: records == frames, nothing dropped
(the 57 % replay-drop gotcha in CLAUDE.md §13 stays fixed).

## 3. What nav would do with it

Chase mode takes `state == CONFIRM` and `action == PICK`, projects the bottom
edge of the box, and picks the nearest.

| | `s640-c` | `n640-c` | `n480-c` |
|---|---|---|---|
| Distinct track ids (his) | 105 | 163 | 211 |
| …of which ever chased (CONFIRM+PICK) | 36 | 38 | 29 |
| Chased target rows | 2634 | 5344 | 5044 |
| Chased targets per frame, median / max | 5 / 13 | 6 / 15 | 7 / 12 |
| Rejected by the projector (horizon / range) | 0 / 0 | 0 / 0 | 0 / 0 |
| Frames with a target to chase | 383/463 | 778/780 | 699/887 |
| Times the chosen target changes | 33 | 33 | 13 |
| …of which between two tracks of one object | 28 | 17 | 7 |
| Most-chosen id (frames) | 43 (142) | 66 (288) | 52 (349) |
| Range under the **fictional** calibration, m | 0.23 – 1.01 | 0.23 – 1.02 | 0.23 – 0.81 |

The same numbers, as input → output, with the rule that decides each step:

| Step | Rule | `s640-c` in → out | `n640-c` in → out | `n480-c` in → out |
|---|---|---|---|---|
| Drop coasting tracks | `misses > 0` means he did not see it this frame; his `latest()` drops it too | 3746 → 3040 | 8079 → 6471 | 8882 → 6965 |
| Keep CONFIRM + PICK | his tracker decides what is real; nav adds no hysteresis | 3040 → 2634 | 6471 → 5344 | 6965 → 5044 |
| Project the bottom-centre | the bottom edge touches the floor; the centroid would over-range | 2634 → 2634 | 5344 → 5344 | 5044 → 5044 |
| Pick the nearest, per frame | cheapest to reach; tracks seen ≤ `max_age_s` ago count | 463 → 383 frames | 780 → 778 frames | 887 → 699 frames |

The replay CLI prints the same picture: `fodnav-replay` output for each log is
in §6 and in each `replay.txt`.

## 4. Findings

**1. The parser and his library agree.** No field name, box convention, frame
size or `latest()` semantics disagree anywhere in 2130 frames. This retires the
first line of the "What is actually blocking" list in `STATUS.md` as far as
format goes. It does not retire the Pi install: these came from his capture
script, not from `LibraryVisionSource.start()`, which has still never run.

**2. Every rule nav was told to follow shows up in the data.**
- `cls` flips on a single track id in 37/105, 60/163 and 73/211 tracks —
  one n480 id reads `bolt`, `nut` and `screw` over its life. *Never branch on
  `cls`* is not theoretical.
- The ground point sits a median 21–23 px below the centroid, up to 102 px on
  an ordinary box (352 px on the whole-frame box of finding 3). That gap is the
  over-range projecting the centroid would have caused.
- Only `CAUTION → CONFIRM` transitions were seen (22 / 24 / 25), never out of
  `CONFIRM`: the latch behaves as documented. `IGNORE` never appears, because
  the 0.25 host filter drops anything that would start there.
- `action` is `PICK` on every row, as documented.

**3. A near-whole-frame box reaches CONFIRM at start-up (`n640-c`).** Frames
3, 4 and 8 each carry a box spanning almost the full image width —
`[5, 0, 1184, 704]` (CAUTION) at frame 3, `[11, 0, 1157, 612]` in **CONFIRM at
0.51** on frame 4, `[14, 2, 1170, 491]` (CAUTION) at frame 8 — and then none
for the rest of the run, which looks like exposure or focus settling rather
than a real object. **Nav chased it:** on frame 4 it is the only target, and
`nav_frames.jsonl` shows nav choosing it, dead ahead (0.27 m, +3° under the
fictional calibration — near, because its bottom edge is low in the image).
Nav has **no box-size sanity gate**. Whether it should is a design decision,
not something to slip in here — a size gate is not re-implementing his
tracking, but the threshold depends on real object sizes at the real mount.
Worth raising with Bthcorn first: the cleanest fix is on his side, by not
reporting frames before the camera has settled.
`tests/test_vision_real.py` pins this frame so the behaviour is on record.

**4. The scene has many targets, and "nearest" switches between them.** 5–7
chaseable targets per frame, and the chosen one changes 13–33 times in ~30 s, with
no robot motion recorded. **Most of those switches (52 of 79) are not between
objects at all**. They are between two tracks of one object; see finding 8.
The rest are real. On a moving robot, nearest-first is stable only while one
object is clearly closer than the rest. That was never exercised in sim, where
scenes have one or two objects. It is not a bug yet, but it is a question for
the chase demo: will the arena ever have more than one fastener in view?

**5. The `s` model runs at 14–20 FPS here.** His thread completed 653 frames
in 32.7 s (~20 FPS); the log holds 463 of them (14 FPS), with an inter-frame
p95 of 152 ms. Chase re-targets every frame, so this roughly halves its update
rate. The `n` models hold 30 FPS; prefer one of them unless
the `s` model's recall is needed.

**6. `blocked` is true 40–67 % of the time.** That is
his `zone_blocked()` with whatever `lookahead` the capture used (not recorded
in `meta.json`), and any `lookahead` is a placeholder until the mount is
measured. Nav does not read it in chase mode; recorded for Bthcorn.

**7. Two small faults in `fodnav-replay`, noticed while doing this** (not
fixed here):
- Its "projected" count is *live tracks per frame summed*, including tracks
  held up to `detections.max_age_s` after their last sighting — so it reads
  3185 projected against 2634 confirmed for `s640-c`, which cannot literally be
  true. The audit counts actual projections (2634).
- Its module docstring still refers to `fodnav.link.detections`, which was
  removed in 0.2.0.

**8. One object is often held as two tracks, with different `cls`.** Two live
tracks overlap at IoU > 0.7 in 201/463, 245/780 and 289/887 frames, both CONFIRM
in 133, 76 and 88 of them. In 905 of 911 such pairs the two copies carry
different `cls` (`s640-c` frame 154: id 1 `screw` and id 11 `bolt`, IoU 0.94).
That pattern points to per-class NMS in his post-processing: the detector
scores one fastener as both `bolt` and `screw`, both boxes survive, and his
80 px association then keeps two tracks. It explains:
- the inflated track count (105–211 ids for a static scene);
- most of nav's target switches (28/33, 17/33, 7/13), which are ties at the
  millimetre between the two copies.

**For steering, it does no harm.** Both copies project to the same floor point, so
a switch between them does not move the servo's target. Where it does harm is
anything keyed on track id: a "caught id X" log, a count of objects seen, and
the hand-off to the blind leg if it ever latches by id. **Nav should not
deduplicate.** That would be a second association stage in series with his, the
thing CLAUDE.md §13 warns against. Ask Bthcorn whether class-agnostic NMS is an
option. The classes are going to one anyway when the single-class arena dataset
lands, which removes the cause.

## 5. Caveats

- Scene unknown. What was in front of the camera, how far, and how many real
  objects there were is not recorded in `meta.json`, so there is no ground truth
  to score detections against. Nothing here says anything about recall or
  precision.
- Whether the camera moved is not recorded; there is no odometry in these logs.
- All metres are from a fictional calibration (above).
- `meta.json` records `focus 0.30`; `camera.focus_m` is `null` in every record.

## 6. `fodnav-replay` output, verbatim

```
s640-c/raw_part01.jsonl: 463 records, 463 frames, 0 carrying an error
  frame sizes      {'1280x720': 463} (calibrated at 1280x720)
  targets          3040 over 463 frames (79 empty, which mean the floor is clear, not that vision died)
      CONFIRM      2634  <- chased
      CAUTION       406
  confirmed+PICK   2634
  projected        3185 across 36 distinct track ids
      rejected: 0 above the horizon or behind, 0 out of range
      range: 0.232 to 1.006 m, median 0.447 m

n640-c/raw_part01.jsonl: 780 records, 780 frames, 0 carrying an error
  frame sizes      {'1280x720': 780} (calibrated at 1280x720)
  targets          6471 over 780 frames (5 empty, which mean the floor is clear, not that vision died)
      CONFIRM      5344  <- chased
      CAUTION      1127
  confirmed+PICK   5344
  projected        6039 across 38 distinct track ids
      rejected: 0 above the horizon or behind, 0 out of range
      range: 0.23 to 1.017 m, median 0.424 m

n480-c/raw_part01.jsonl: 887 records, 887 frames, 0 carrying an error
  frame sizes      {'1280x720': 887} (calibrated at 1280x720)
  targets          6965 over 887 frames (14 empty, which mean the floor is clear, not that vision died)
      CONFIRM      5044  <- chased
      CAUTION      1921
  confirmed+PICK   5044
  projected        5592 across 29 distinct track ids
      rejected: 0 above the horizon or behind, 0 out of range
      range: 0.23 to 0.808 m, median 0.363 m
```
