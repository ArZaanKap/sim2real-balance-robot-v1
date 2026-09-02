# BAM fit results — model comparison bench

Experiments comparing actuator models on the **same** captured logs, per motor.
The **canonical** fit each motor uses in sim lives one level up as
`ga25-370_motorN_params.json`; everything here is the playground behind that choice.

## Layout
```
BAM/fit/
  ga25-370_motorN_run.csv       # raw capture (one per motor, tracked)
  ga25-370_motorN_params.json   # the CHOSEN fit the sim uses (tracked)
  fits/
    motorN_mM.json              # each model experiment
    motorN_mM.png               # its plot_fit overlay
    RESULTS.md                  # this table
```
Regenerate any row:
```bash
python3 fit_freeshaft.py --logs logs/ --model mM --kt 0.24 --R 6.65 --maxiter 60 --out fits/motorN_mM.json
python3 plot_fit.py --logs logs/ --params fits/motorN_mM.json --save fits/motorN_mM.png
```
(`logs/` is scratch — rebuild from the tracked CSV with `csv_to_bam.py` first.)

## Motor #1 (GA25-370)  — capture `ga25-370_motor1_run.csv`, vin 11.97 V, kt 0.24, R 6.65

| model | terms added | avg MAE (rad/s) | verdict |
|-------|-------------|-----------------|---------|
| **m1** | Coulomb + viscous | **0.4127** | **CANONICAL.** powered + deadband tight; coast-down over-braked |
| m2 | + Stribeck | 0.4112 (−0.4%) | Stribeck does NOT fix the coast-down — same residual spikes. Not worth it |
| m3 | + load-dependent | 0.4127 (0%) | zero improvement; `load_friction_base`=0.017 is noise → UNidentifiable free-shaft (as predicted) |

**Conclusion (2026-09-01): keep m1.** Neither Stribeck (m2) nor load-dependence (m3) earns its
keep on this data. The **coast-down over-brake is structural**, not a low-speed-friction problem —
the model brakes a high-speed free coast with the same Coulomb+viscous that pins the powered
steady-state, and no term in the m1–m3 family resolves that tension. Likely causes: real coast
friction < powered friction (a load effect our free shaft can't observe), or armature/inertia
slightly low. Not worth chasing for balancing (deadband + viscous are what matter, and both are
nailed). MAE is dominated by the two coast transients, which is why all three look ~identical.

Rule of thumb this confirmed: only adopt a heavier model if the overlay/MAE **clearly** improves —
else keep the simplest (fewer params = less to randomize, less overfit). See `bam-model-zoo`.
