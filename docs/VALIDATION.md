# Validation

Run `bash scripts/check_host.sh` for C++ and Python tests. The runner must
exist at `build/sil_runner`; otherwise Python skips the socket tests.
Build the runner first so the socket checks run.

Run `python3 sim/monte_carlo.py --runs 40 --workers 2 --out runs/mc` for the
simulation study. `summary.json` records settings, seeds, and metric summaries.
The existing figures and report describe the supplied 40-run study; a rerun
should be kept with its own results rather than silently replacing that report.

Use `--realtime` only when measuring wall-clock behavior. Lockstep tests check
functional behavior and permit the host to run faster or slower than real time.
On hardware, collect maximum control-step time, link latency, period overruns,
stack headroom, and queue drops. Hardware testing remains pending.
