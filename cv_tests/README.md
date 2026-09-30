# Real output from the CV repo, run through nav

One directory per capture session, named by date. Raw input goes in `data/`,
exactly as it came off Bthcorn's board; everything nav made of it goes in
`output/`.

```
cv_tests/<date>/
  data/<capture>/          meta.json + raw_part*.jsonl from his scripts/capture_raw.py
  output/summary.md        the cross-capture comparison and findings -- start here
  output/<capture>/
    summary.md             verdict for this capture, compared with the others
    audit.json             the same numbers, machine-readable
    nav_frames.jsonl       nav's output for every input frame
    replay.txt             fodnav-replay on the same log, verbatim
```

For a new session, drop his capture folders into `cv_tests/<date>/data/` and run:

```bash
uv run python tools/audit_cv_capture.py cv_tests/<date>
```

That writes every `output/<capture>/` file. `output/summary.md` is written by
hand, because the findings need judgement.

Any metres in these outputs come from the fictional sim calibration until the
real camera mount is measured (PRD O-3). They are not where the objects were.
