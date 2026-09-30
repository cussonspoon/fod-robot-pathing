# s640-c — what nav made of it

Input: `../../data/s640-c/`. Model `arg-bolts-4-s-640/bench_int8_hailo_model`, imgsz 640, host conf 0.25.

## Verdict

**Parsable.** 463/463 lines parsed, 0 kinds of schema problem, and nav's `latest()` matched his `targets` on 463/463 frames (3040 targets ours, 3040 his).

## Input → output

What goes into nav from his log, what comes out, and the rule that decides each step. Metres use the **fictional** sim calibration; they show the code path, not where anything was.

### The whole capture

| Step | In (his data) | Out (nav) | Why |
|---|---|---|---|
| Read the log | 463 lines | 463 frames | Each line is one `detail()`; nav's parser, `parse_detail`, reads it directly. |
| Drop coasting tracks | 3746 track rows | 3040 rows (his `targets`: 3040) | A coasting track (`misses` > 0) was not seen this frame. His `latest()` drops them; nav rebuilds the same list from `tracks`. |
| Keep CONFIRM + PICK | 3040 rows | 2634 rows, 36 ids | His tracker decides what is real (CONFIRM at 0.5, latched to 0.25). Nav adds no hysteresis of its own. |
| Project to the floor | 2634 boxes | 2634 floor points, 0 rejected | Bottom-centre of the box, not the centroid: the bottom edge is where the object touches the floor. |
| Pick one | 463 frames | 383 frames with a target | Nearest floor point, including tracks last seen ≤ `max_age_s` ago. Nearest is cheapest to reach. |

### Frame 154 — a typical frame, with as many kinds of row as possible

**In:** `../../data/s640-c/raw_part01.jsonl:120`, 5 tracks, `frame_size` 1280×720.  
**Out:** drive at id **1**, 0.584 m at +7.4°.

| id | state | misses | conf | box (px) | → | nav | ground pt (px) | floor x, y (m) | range, bearing | why |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | CONFIRM | 0 | 0.54 | `[477, 209, 535, 253]` | → | **chosen** | (506, 253) | 0.579, +0.075 | 0.584 m, +7.4° | CONFIRM + PICK, and the nearest of 3 candidate(s), so nav drives at it. Projected from the bottom-centre, 22 px below the centroid. **Same object as id 11** (IoU 0.94, `screw` vs `bolt`): his tracker holds one object as two tracks. |
| 11 | CONFIRM | 0 | 0.60 | `[477, 207, 534, 253]` | → | candidate | (506, 253) | 0.579, +0.076 | 0.584 m, +7.5° | CONFIRM + PICK, projected from the bottom-centre. Not chosen: id 1 is nearer (0.584 m against 0.584 m, a tie at the millimetre). **Same object as id 1** (IoU 0.94, `bolt` vs `screw`): his tracker holds one object as two tracks. |
| 12 | CAUTION | 0 | 0.36 | `[490, 137, 546, 175]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.36; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |
| 13 | CAUTION | 0 | 0.31 | `[410, 177, 499, 211]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.31; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |
| 14 | CAUTION | 2 | 0.31 | `[413, 177, 502, 212]` | → | dropped (coasting) | — | — | — | His tracker did not see it this frame (`misses` 2). His `latest()` leaves coasting tracks out, and so does nav. |
| 2 | — | — | — | *not in this frame* | → | held | — | 0.712, -0.137 | 0.725 m, -10.9° | Gone from his list, but last seen as CONFIRM 0.299 s ago, inside `detections.max_age_s`, so it is still a candidate. |

## What nav would do

- Chase mode (CONFIRM + PICK) had a target in **383/463** frames, from 36 of his 105 track ids.
- Projector rejected 0 of 2634 chased targets.
- Most-chosen targets (id: frames): 43: 142, 1: 130, 58: 65, 11: 30, 0: 9.
- Range 0.232 / 0.445 / 1.006 m (min / median / max) — **fictional calibration, not a measurement.**

## Worth knowing

- Logged at **14.1 FPS**; his thread completed frames 1→653 and 190 are not in the log. Infer 45.3 / 47.1 ms (median / p95).
- `cls` flips on 37/105 track ids — never branch on it.
- State transitions seen: CAUTION → CONFIRM ×22.
- **One object, two tracks:** a pair of live tracks overlap at IoU > 0.7 in 201/463 frames (133 with both CONFIRM); 257 of 260 such pairs carry different `cls`. Of nav's 33 target switches, **28 are between two tracks of the same object.**
- `blocked` true on 187/463 frames (nav ignores it in chase mode).
- Extra top-level keys not in the `detail()` spec: `t`, `targets` (added by `capture_raw.py`; nav ignores them).

## Against the other captures in this session

| | n480-c | n640-c | s640-c |
|---|---|---|---|
| Frames parsed | 887/887 | 780/780 | **463/463** |
| `latest()` matches his | 887/887 | 780/780 | **463/463** |
| Logged FPS | 29.9 | 29.7 | **14.1** |
| Infer ms, median / p95 | 10.9 / 11.5 | 16.0 / 16.7 | **45.3 / 47.1** |
| Distinct track ids | 211 | 163 | **105** |
| …ever chased | 29 | 38 | **36** |
| Frames with a target | 699/887 | 778/780 | **383/463** |
| `cls` flips (ids) | 73 | 60 | **37** |
| Frames with a same-object duplicate | 289/887 | 245/780 | **201/463** |
| Target switches, same-object / all | 7/13 | 17/33 | **28/33** |
| Whole-frame CONFIRM boxes | 0 | 1 | **0** |

Files here: `audit.json` (these numbers), `nav_frames.jsonl` (nav's output per input frame), `replay.txt` (`fodnav-replay` verbatim). The cross-capture write-up is `../summary.md`.
