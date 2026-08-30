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

## Phase 2 — fit (PC, not started)

Install BAM, convert `run.csv` to its expected trajectory format, fit with CMA-ES. Start with model
**M3** (Coulomb + viscous), escalate to **M5/M6** (adds back-EMF + Stribeck) only if the velocity
residuals are bad. We have **no current sensor** (DRV8871), so pin the electrical params by hand
(R from stall current, Ke from no-load speed) and let BAM fit the friction terms — see the
`bam-ga25-identification-plan` memory. Fit per-motor or fit one and randomize the spread.

## Phase 3 — into MuJoCo Warp (not started)

Use BAM's MuJoCo/Warp API to install the fitted model as the wheel actuator, then wrap ±20–30%
domain randomization around the fitted params. The MJCF itself is hand-written from Fusion mass
properties — see the `cad-to-mujoco` memory.

## For a balancer, the two params that matter most

**Deadband/static friction** (kills fine low-speed corrections → wobble) and **viscous** drag. Nail
those two and the transfer is most of the way there.
