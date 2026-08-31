# balancing-2-wheel-robot — project rules & runbook

A 2-wheel self-balancing robot. ESP32-only (no Pi), ESP-IDF toolchain. RL policy trained
in MuJoCo (PPO) with sim2real onto the MCU. Sister project: `../home-rover`.

## GIT — hard rule (do not violate)

**Only the human runs git.** Claude must **NEVER** run `git add`, `git commit`, `git push`,
`git reset`, `git checkout <ref>`, `git rebase`, `git stash`, or any command that stages,
commits, moves, or rewrites history — **not even when it looks helpful, and not when asked to
"prepare a commit."** Claude may: edit/create/delete working-tree files, and run **read-only**
git (`git status`, `git diff`, `git log`). To hand off, Claude lists the changed files and a
suggested commit message as text; the human does the `add`/`commit`/`push`.

## ESP-IDF: build / flash / monitor

ESP-IDF **v5.5.3** lives in the sister project with a **project-local** tools path, so a bare
`export.sh` fails. Use the helper (details + the gotcha are in `idf-env.sh`):

```bash
source idf-env.sh                 # sets IDF_TOOLS_PATH + IDF_PATH, then export.sh
cd encoder_test                   # or motor_test, or BAM/sweep_logger
idf.py build                      # Claude CAN do this (verifies compile)
idf.py -p /dev/ttyUSB0 flash monitor   # HUMAN step — needs the board attached
```

**WSL USB (human step):** the board is a CH340 (`1a86:7523`). In Windows PowerShell first:
`usbipd attach --wsl --hardware-id 1a86:7523` (one-time `usbipd bind` as admin). Then it
appears as `/dev/ttyUSB0`. Re-attach only after a reboot / `wsl --shutdown` / replug.

## BAM friction-model fit (Phase 2 pipeline)

Needs `bam` (installed: `pip install --break-system-packages better-actuator-models`) + numpy + scipy.
From `BAM/fit/`, per motor:

```bash
# 1) ESP32: flash BAM/sweep_logger, capture the pure CSV:
#    stty -F /dev/ttyUSB0 115200 raw && cat /dev/ttyUSB0 | grep --line-buffered '^BAM,' > ga25-370_motorN_run.csv
# 2) convert (auto-flips inverted encoder sign; --stride downsamples for speed):
python3 csv_to_bam.py ga25-370_motorN_run.csv logs/ --counts-per-rev 1976 --vin 11.97 --stride 4
# 3) fit (pin kt/R from datasheet STALL, not no-load speed — see the memory):
python3 fit_freeshaft.py --logs logs/ --model m1 --kt 0.24 --R 6.65 --maxiter 60 --out ga25-370_motorN_params.json
# prove the fitter with no hardware:
python3 fit_freeshaft.py --selftest
```

Motor #1 result (committed): `ga25-370_motor1_run.csv` + `ga25-370_motor1_params.json`
(model m1: friction_base 0.228 Nm, friction_viscous 0.0056, armature 7.1e-4). `logs/` and
`run*.csv`/`params*.json` are scratch (gitignored); the `ga25-370_motorN_*` files are tracked.

## Calibration constants (measured, in firmware)

- `COUNTS_PER_REV = 1976` (13 poles ×4 quadrature ×38:1 gearbox; RPM-confirmed at 133≈130).
- `V_BUS_VOLTS = 11.97` (bench supply set 12.0, measured at VM under load). Cosmetic in
  firmware (header print only); the real value is passed to `csv_to_bam.py --vin`.
