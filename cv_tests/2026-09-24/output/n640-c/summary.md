# n640-c — what nav made of it

Input: `../../data/n640-c/`. Model `arg-bolts-4-n-640/bench_int8_hailo_model_conf00001`, imgsz 640, host conf 0.25.

## Verdict

**Parsable.** 780/780 lines parsed, 0 kinds of schema problem, and nav's `latest()` matched his `targets` on 780/780 frames (6471 targets ours, 6471 his).

## Input → output

What goes into nav from his log, what comes out, and the rule that decides each step. Metres use the **fictional** sim calibration; they show the code path, not where anything was.

### The whole capture

| Step | In (his data) | Out (nav) | Why |
|---|---|---|---|
| Read the log | 780 lines | 780 frames | Each line is one `detail()`; nav's parser, `parse_detail`, reads it directly. |
| Drop coasting tracks | 8079 track rows | 6471 rows (his `targets`: 6471) | A coasting track (`misses` > 0) was not seen this frame. His `latest()` drops them; nav rebuilds the same list from `tracks`. |
| Keep CONFIRM + PICK | 6471 rows | 5344 rows, 38 ids | His tracker decides what is real (CONFIRM at 0.5, latched to 0.25). Nav adds no hysteresis of its own. |
| Project to the floor | 5344 boxes | 5344 floor points, 0 rejected | Bottom-centre of the box, not the centroid: the bottom edge is where the object touches the floor. |
| Pick one | 780 frames | 778 frames with a target | Nearest floor point, including tracks last seen ≤ `max_age_s` ago. Nearest is cheapest to reach. |

### Frame 184 — a typical frame, with as many kinds of row as possible

**In:** `../../data/n640-c/raw_part01.jsonl:180`, 5 tracks, `frame_size` 1280×720.  
**Out:** drive at id **0**, 0.536 m at -5.6°.

| id | state | misses | conf | box (px) | → | nav | ground pt (px) | floor x, y (m) | range, bearing | why |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | CONFIRM | 0 | 0.56 | `[717, 236, 762, 281]` | → | **chosen** | (740, 281) | 0.534, -0.052 | 0.536 m, -5.6° | CONFIRM + PICK, and the nearest of 4 candidate(s), so nav drives at it. Projected from the bottom-centre, 22 px below the centroid. |
| 1 | CONFIRM | 0 | 0.48 | `[486, 194, 545, 247]` | → | candidate | (516, 247) | 0.589, +0.071 | 0.594 m, +6.9° | CONFIRM + PICK, projected from the bottom-centre. Not chosen: id 0 is nearer (0.536 m against 0.594 m). Conf 0.48 is under 0.5, but his latch holds CONFIRM until 0.25. |
| 8 | CONFIRM | 0 | 0.49 | `[643, 141, 687, 168]` | → | candidate | (665, 168) | 0.773, -0.019 | 0.773 m, -1.4° | CONFIRM + PICK, projected from the bottom-centre. Not chosen: id 0 is nearer (0.536 m against 0.773 m). Conf 0.49 is under 0.5, but his latch holds CONFIRM until 0.25. |
| 43 | CAUTION | 0 | 0.30 | `[673, 197, 726, 224]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.30; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |
| 46 | CAUTION | 1 | 0.28 | `[630, 107, 662, 130]` | → | dropped (coasting) | — | — | — | His tracker did not see it this frame (`misses` 1). His `latest()` leaves coasting tracks out, and so does nav. |
| 40 | — | — | — | *not in this frame* | → | held | — | 0.903, -0.005 | 0.903 m, -0.3° | Gone from his list, but last seen as CONFIRM 0.496 s ago, inside `detections.max_age_s`, so it is still a candidate. |

### Frame 4 — the whole-frame box that reaches CONFIRM

**In:** `../../data/n640-c/raw_part01.jsonl:3`, 2 tracks, `frame_size` 1280×720.  
**Out:** drive at id **1**, 0.27 m at +3.3°.

| id | state | misses | conf | box (px) | → | nav | ground pt (px) | floor x, y (m) | range, bearing | why |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | CAUTION | 2 | 0.40 | `[665, 439, 722, 476]` | → | dropped (coasting) | — | — | — | His tracker did not see it this frame (`misses` 2). His `latest()` leaves coasting tracks out, and so does nav. |
| 1 | CONFIRM | 0 | 0.51 | `[11, 0, 1157, 612]` | → | **chosen** | (584, 612) | 0.270, +0.016 | 0.270 m, +3.3° | CONFIRM + PICK, and the nearest of 1 candidate(s), so nav drives at it. Projected from the bottom-centre, 306 px below the centroid. **The box spans 90% of the frame width** -- not a fastener, but nav has no size gate. |

## What nav would do

- Chase mode (CONFIRM + PICK) had a target in **778/780** frames, from 38 of his 163 track ids.
- Projector rejected 0 of 5344 chased targets.
- Most-chosen targets (id: frames): 66: 288, 1: 157, 0: 129, 87: 72, 96: 71.
- Range 0.23 / 0.423 / 1.017 m (min / median / max) — **fictional calibration, not a measurement.**

## Worth knowing

- Logged at **29.7 FPS**; his thread completed frames 2→788 and 7 are not in the log. Infer 16.0 / 16.7 ms (median / p95).
- `cls` flips on 60/163 track ids — never branch on it.
- State transitions seen: CAUTION → CONFIRM ×24.
- **1 CONFIRM box(es) wider than 80% of the frame**, first at frame 4 (id 1, conf 0.514, box [11, 0, 1157, 612]). Nav has no size gate and will chase it.
- **One object, two tracks:** a pair of live tracks overlap at IoU > 0.7 in 245/780 frames (76 with both CONFIRM); 288 of 288 such pairs carry different `cls`. Of nav's 33 target switches, **17 are between two tracks of the same object.**
- `blocked` true on 466/780 frames (nav ignores it in chase mode).
- Extra top-level keys not in the `detail()` spec: `t`, `targets` (added by `capture_raw.py`; nav ignores them).

## Against the other captures in this session

| | n480-c | n640-c | s640-c |
|---|---|---|---|
| Frames parsed | 887/887 | **780/780** | 463/463 |
| `latest()` matches his | 887/887 | **780/780** | 463/463 |
| Logged FPS | 29.9 | **29.7** | 14.1 |
| Infer ms, median / p95 | 10.9 / 11.5 | **16.0 / 16.7** | 45.3 / 47.1 |
| Distinct track ids | 211 | **163** | 105 |
| …ever chased | 29 | **38** | 36 |
| Frames with a target | 699/887 | **778/780** | 383/463 |
| `cls` flips (ids) | 73 | **60** | 37 |
| Frames with a same-object duplicate | 289/887 | **245/780** | 201/463 |
| Target switches, same-object / all | 7/13 | **17/33** | 28/33 |
| Whole-frame CONFIRM boxes | 0 | **1** | 0 |

Files here: `audit.json` (these numbers), `nav_frames.jsonl` (nav's output per input frame), `replay.txt` (`fodnav-replay` verbatim). The cross-capture write-up is `../summary.md`.
