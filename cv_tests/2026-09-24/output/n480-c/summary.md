# n480-c — what nav made of it

Input: `../../data/n480-c/`. Model `arg-bolts-4-n-640/bench_int8_hailo_model_480`, imgsz 480, host conf 0.25.

## Verdict

**Parsable.** 887/887 lines parsed, 0 kinds of schema problem, and nav's `latest()` matched his `targets` on 887/887 frames (6965 targets ours, 6965 his).

## Input → output

What goes into nav from his log, what comes out, and the rule that decides each step. Metres use the **fictional** sim calibration; they show the code path, not where anything was.

### The whole capture

| Step | In (his data) | Out (nav) | Why |
|---|---|---|---|
| Read the log | 887 lines | 887 frames | Each line is one `detail()`; nav's parser, `parse_detail`, reads it directly. |
| Drop coasting tracks | 8882 track rows | 6965 rows (his `targets`: 6965) | A coasting track (`misses` > 0) was not seen this frame. His `latest()` drops them; nav rebuilds the same list from `tracks`. |
| Keep CONFIRM + PICK | 6965 rows | 5044 rows, 29 ids | His tracker decides what is real (CONFIRM at 0.5, latched to 0.25). Nav adds no hysteresis of its own. |
| Project to the floor | 5044 boxes | 5044 floor points, 0 rejected | Bottom-centre of the box, not the centroid: the bottom edge is where the object touches the floor. |
| Pick one | 887 frames | 699 frames with a target | Nearest floor point, including tracks last seen ≤ `max_age_s` ago. Nearest is cheapest to reach. |

### Frame 204 — a typical frame, with as many kinds of row as possible

**In:** `../../data/n480-c/raw_part01.jsonl:199`, 5 tracks, `frame_size` 1280×720.  
**Out:** drive at id **19**, 0.471 m at -11.1°.

| id | state | misses | conf | box (px) | → | nav | ground pt (px) | floor x, y (m) | range, bearing | why |
|---|---|---|---|---|---|---|---|---|---|---|
| 3 | CAUTION | 0 | 0.41 | `[765, 242, 824, 268]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.41; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |
| 19 | CONFIRM | 0 | 0.47 | `[815, 286, 864, 336]` | → | **chosen** | (840, 336) | 0.462, -0.091 | 0.471 m, -11.1° | CONFIRM + PICK, and the nearest of 1 candidate(s), so nav drives at it. Projected from the bottom-centre, 25 px below the centroid. Conf 0.47 is under 0.5, but his latch holds CONFIRM until 0.25. |
| 32 | CAUTION | 0 | 0.42 | `[732, 176, 779, 205]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.42; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |
| 34 | CAUTION | 1 | 0.30 | `[955, 209, 1002, 230]` | → | dropped (coasting) | — | — | — | His tracker did not see it this frame (`misses` 1). His `latest()` leaves coasting tracks out, and so does nav. |
| 35 | CAUTION | 0 | 0.35 | `[594, 156, 646, 195]` | → | ignored (CAUTION) | — | — | — | In `latest()`, but his tracker has not confirmed it (conf 0.35; CONFIRM needs 0.5). Nav chases CONFIRM only and adds no hysteresis of its own. |

## What nav would do

- Chase mode (CONFIRM + PICK) had a target in **699/887** frames, from 29 of his 211 track ids.
- Projector rejected 0 of 5044 chased targets.
- Most-chosen targets (id: frames): 52: 349, 3: 127, 135: 62, 42: 51, 72: 51.
- Range 0.23 / 0.368 / 0.808 m (min / median / max) — **fictional calibration, not a measurement.**

## Worth knowing

- Logged at **29.9 FPS**; his thread completed frames 3→892 and 3 are not in the log. Infer 10.9 / 11.5 ms (median / p95).
- `cls` flips on 73/211 track ids — never branch on it.
- State transitions seen: CAUTION → CONFIRM ×25.
- **One object, two tracks:** a pair of live tracks overlap at IoU > 0.7 in 289/887 frames (88 with both CONFIRM); 360 of 363 such pairs carry different `cls`. Of nav's 13 target switches, **7 are between two tracks of the same object.**
- `blocked` true on 594/887 frames (nav ignores it in chase mode).
- Extra top-level keys not in the `detail()` spec: `t`, `targets` (added by `capture_raw.py`; nav ignores them).

## Against the other captures in this session

| | n480-c | n640-c | s640-c |
|---|---|---|---|
| Frames parsed | **887/887** | 780/780 | 463/463 |
| `latest()` matches his | **887/887** | 780/780 | 463/463 |
| Logged FPS | **29.9** | 29.7 | 14.1 |
| Infer ms, median / p95 | **10.9 / 11.5** | 16.0 / 16.7 | 45.3 / 47.1 |
| Distinct track ids | **211** | 163 | 105 |
| …ever chased | **29** | 38 | 36 |
| Frames with a target | **699/887** | 778/780 | 383/463 |
| `cls` flips (ids) | **73** | 60 | 37 |
| Frames with a same-object duplicate | **289/887** | 245/780 | 201/463 |
| Target switches, same-object / all | **7/13** | 17/33 | 28/33 |
| Whole-frame CONFIRM boxes | **0** | 1 | 0 |

Files here: `audit.json` (these numbers), `nav_frames.jsonl` (nav's output per input frame), `replay.txt` (`fodnav-replay` verbatim). The cross-capture write-up is `../summary.md`.
