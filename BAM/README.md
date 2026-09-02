# BAM — actuator-model identification for the GA25-370

Goal: give MuJoCo a **realistic motor** instead of its dumb default one, so the balancing policy
trained in sim transfers to the real robot. We do this with **BAM** (Rhoban, "Extended Friction
Models for Servo Actuators", ICRA 2025 — <https://github.com/Rhoban/bam>).

## Why this exists (the one-paragraph version)

MuJoCo's default actuator is `torque = gain × command` — instant, linear, frictionless. A real
GA25-370 gearmotor has static friction (a deadband before it moves), viscous drag, Stribeck effects,
and back-EMF (less torque the faster it spins). If the sim motor is too clean, the policy learns to
exploit that and falls over on hardware. BAM fits a better `torque = f(voltage, wheel_speed)` to
**our** motor and drops it into MuJoCo (it ships a MuJoCo **and MuJoCo Warp** API), closing the
actuator half of the sim-to-real gap. The sensor half (BNO085 latency, encoder noise) is handled
separately by domain randomization.

## Pipeline

```
[ESP32 sweep_logger]  ->  run.csv  ->  [BAM fit on PC]  ->  params  ->  [MuJoCo Warp wheel actuator]
   drive + measure        voltage,        CMA-ES fits        a few         realistic torque during
   (this folder)          speed logs      the model          numbers       RL training
```

## Phase 1 — capture (this folder: `sweep_logger/`)  ← we are here

ESP-IDF firmware that drives one motor + reads its encoder and prints CSV. Three sweeps:

| sweep | what it does | what it isolates |
|-------|--------------|------------------|
| **staircase** | hold duties 60→255 both directions | steady-state speed vs voltage → viscous + Stribeck; spin-up transient → inertia |
| **spindown** | spin up, then coast (power cut) | **friction only**, no drive torque — the cleanest friction signal |
| **deadband** | creep duty up from ~0 | breakaway duty = static friction / start-up deadband (matters most for fine balance) |

Build / flash / capture:
```bash
cd sweep_logger
idf.py set-target esp32
idf.py -p /dev/ttyUSB0 flash
# log only the pure CSV rows to a file:
stty -F /dev/ttyUSB0 115200 raw && cat /dev/ttyUSB0 | grep --line-buffered '^BAM,' > run.csv
# press EN/RST on the board to start a run; stop cat after you see "BAM,DONE".
```

CSV columns: `BAM, t_ms, phase, duty(-255..255), pos_counts, vel_rad_s`.

**Before trusting the numbers, set two constants in `sweep_logger.c`:**
- `COUNTS_PER_REV` — calibrate by hand exactly like `encoder_test.c` (one output-shaft turn).
- `V_BUS_VOLTS` — measure the pack voltage under load. BAM works in volts: `voltage = duty/255 × V_BUS`.

Safety: open-loop only (no runaway possible), bare shaft, bring 12 V up **after** flashing.

## Phase 2 — fit (PC, tooling in `fit/`, self-test passing)

> **Why not `python -m bam.fit`?** BAM's stock fitter is built for a smart *servo on a
> pendulum*: it replays a **position-goal** trajectory through the servo's P-controller
> (`simulate_control=True`) and scores **position error** against a gravity load
> (`mass·g·length·sin q`). Our data is **open-loop voltage on a free-spinning shaft** — no
> goal, no gravity load, and position drifts unboundedly so position-matching is the wrong
> objective. So we keep BAM's real friction math + MuJoCo/Warp export and drive its own
> `Simulator` with `simulate_control=False` (which replays our recorded **voltage** straight
> into `τ = kt·V/R − kt²·ω/R`), matching **velocity** instead. Two small swaps make this work,
> both in `fit/freeshaft.py`:
> - `FreeShaft` testbench — no gravity bias, inertia lives in the motor `armature`.
> - `GA25Actuator` — BAM's `VoltageControlledActuator` (already our exact plant) + an `armature`
>   inertia param, exactly like `MXActuator`. The firmware P-controller is never called.

Pipeline:
```bash
cd fit
# 1) convert the capture to BAM's JSON log format (three logs, one per sweep):
python csv_to_bam.py ../run.csv logs/ --counts-per-rev 1976 --vin 11.97
# 2) fit friction, pinning the electrical params by hand (Plan A — no current sensor):
python fit_freeshaft.py --logs logs/ --model m1 --kt 0.24 --R 6.65   # -> params.json
# prove the fitter with no hardware at all:
python fit_freeshaft.py --selftest
```

**Look at the fit (needs `matplotlib`):**
```bash
# fitted vs measured velocity, one panel per sweep + residuals (no hardware):
python plot_fit.py --logs logs/ --params ga25-370_motor1_params.json   # --save fit.png if headless
# watch a sweep live as it runs (velocity + position, rolling window):
python live_plot.py --port /dev/ttyUSB0
#   or pipe the same stream you capture with:
#   stty -F /dev/ttyUSB0 115200 raw && cat /dev/ttyUSB0 | python live_plot.py
```
`plot_fit.py` replays the recorded voltage through the fitted model exactly the way the
fitter scored it, so lines that overlap = a good fit. On Motor #1 the powered plateaus and
the deadband track tightly, but the **coast-down decel is over-braked** by M1's Coulomb+viscous
friction (residual spikes at every duty-to-0 transition) — a known M1 limitation, not a bug.
Pin the electrical params by hand from the datasheet **STALL** point (**R** = V_stall / I_stall,
**kt≈Ke** = stall_torque / I_stall — NOT the no-load speed, which understates both); the friction
terms are fitted. Start at **M1** (Coulomb + viscous) — for free-shaft/balancing that's the right
default, since the self-test showed M3's load term is unidentifiable without an external load.
Escalate to **M2** or **M5/M6** (Stribeck) only if velocity residuals stay bad. Fit per-motor or
fit one and randomize the spread.

Needs `bam`, `numpy`, `scipy` (no optuna/wandb/mujoco for the fit itself). See the
`bam-ga25-identification-plan` memory. **Model numbers (verified against the repo):** M1=Coulomb,
M2=Stribeck, M3=load-dependent; `friction_base` (static) and `friction_viscous` (viscous) are base
terms present in *every* model — so our two priority terms are always fitted.

**Known limitation (from the self-test):** free-shaft data with **no external load** identifies
static + viscous friction well (~5%), but M3's **load-dependence** term (`load_friction_base`) is
weakly constrained — it multiplies `|motor_torque|` here, which correlates with the base terms. For
balancing that is fine (static + viscous are what matter); add a known load if you ever need the
load term pinned.

## Phase 3 — into MuJoCo Warp (not started; unblocked by Phase 2)

The fitted object is a real BAM `Model`, so use BAM's export directly:
`bam.to_mujoco.voltage_controlled_to_mujoco(actuator)` for MuJoCo CPU, or `bam/mjlab.py`
(`import mujoco_warp`) for the Warp training path. Install the fitted model as the wheel actuator,
then wrap ±20–30% domain randomization around the fitted params. The MJCF itself is hand-written
from Fusion mass properties — see the `cad-to-mujoco` memory.

## For a balancer, the two params that matter most

**Deadband/static friction** (kills fine low-speed corrections → wobble) and **viscous** drag. Nail
those two and the transfer is most of the way there.
