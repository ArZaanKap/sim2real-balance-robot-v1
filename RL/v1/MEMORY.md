# Balancing RL session memory — 2026-09-18

This file records the reasoning, code changes, results, and next steps from the
run 1–10 review. It is intended to let a future session resume without
reconstructing the TensorBoard data and Git history.

## Working principle

Keep the balancing objective minimal. Begin with uprightness only, and add a
reward term only after a measured failure shows why it is needed. Prefer
removing unnecessary freedoms from the task over correcting them with reward
penalties.

## Findings from runs 1–8

- `gamma=0.995` was better than `0.99` and `0.997` for this task.
- `normalize_advantage=True` is required. Run 5 failed badly when it was
  disabled.
- More training is not necessarily better. Run 4 reproduced run 2 through
  about 400k steps and then regressed.
- Best-checkpoint evaluation is necessary; final weights can be substantially
  worse.
- Run 6 was the strongest old final policy, but its two motor commands strongly
  disagreed even though the task contained no yaw observation or steering goal.
- Run 7's best checkpoint was smoother, but less robust.
- The old policies use 22 observations and 2 actions. They are incompatible
  with the current environment.

## Changes made before run 9

### Wheel/floor friction

`RL/models/model1.xml` now gives the floor:

```xml
friction="0.1 0.005 0.0001"
```

MuJoCo uses the element-wise maximum of equal-priority geom friction values.
Previously the floor's default sliding coefficient of `1.0` prevented wheel
samples below `1.0` from reaching the actual contact. The low floor coefficient
allows the wheel coefficient to govern contact.

The environment samples shared wheel sliding friction from `0.7–1.1`. Both
wheels share the sample because they contact the same floor. With the current
default `condim=3`, only sliding friction is active; torsional and rolling
friction should not be randomized yet.

### Shared balance action

The policy now emits one action:

```text
u in [-1, 1]
left motor = u
right motor = u
```

This structurally removes unwanted steering instead of introducing a steering
reward. The action space is `(1,)`; the observation space is now `(19,)`:

```text
4 current observations
3 previous shared actions
12 previous observations
```

The environment checker passed and a test command of `0.5` produced controls
`[0.5, 0.5]` after the randomized action delay.

### Minimal reward

The current reward is only:

```python
upright = 1.0 - (gt_pitch / fall_angle) ** 2
reward = upright
```

There is no action-magnitude or action-rate penalty. Falling terminates the
episode, so loss of future upright reward supplies the survival incentive.

### Evaluation

- Training environments: 8, seeded with `0`.
- Evaluation environment: 1, seeded with `10_000`.
- Evaluation: 50 deterministic episodes every 40k training samples.
- Each best policy is saved with normalization statistics from the same point.
- The 50 episodes improve checkpoint selection but add substantial wall time.
- Evaluation uses a reproducible sequence, but not the exact same 50 reset
  seeds at every checkpoint. A separate fixed-seed comparison is still needed.

## Run 9: wide initial-state distribution

Configuration:

```text
initial pitch      = uniform(-0.30, 0.30) rad
initial pitch rate = uniform(-0.30, 0.30) rad/s
```

TensorBoard result:

```text
best checkpoint: 520k steps
50-episode mean reward: 1491.818
50-episode mean length: 1497.00 / 2000 steps = 14.97 s
final mean length: 1298.24 steps = 12.98 s
```

A separate deterministic test of `best9` over fixed seeds `0–299` found:

```text
mean survival:        12.74 s
median survival:      20.00 s
20-second success:    59.7%
fell within 1 second: 29.0%
pitch RMS:            2.44 degrees
mean absolute end x:  0.357 m
command-change RMS:   0.705
```

Failures were strongly structured by initial pitch:

| Initial absolute pitch | 20 s success | Fell under 1 s |
|---|---:|---:|
| 0–0.05 rad | 92.7% | 0.0% |
| 0.05–0.10 rad | 88.5% | 0.0% |
| 0.10–0.15 rad | 83.3% | 6.2% |
| 0.15–0.20 rad | 65.4% | 11.5% |
| 0.20–0.25 rad | 22.7% | 61.4% |
| 0.25–0.30 rad | 5.6% | 94.4% |

Starts moving away from upright failed within one second 35.9% of the time,
versus 21.8% for starts moving toward upright. The wide reset distribution was
therefore mixing local balance with frequently unrecoverable recovery tasks.

## Run 10: structured local-balance starts

Only the initial-state distribution changed:

```python
pitch0 = self.np_random.uniform(-0.10, 0.10)
pitch_rate0 = self.np_random.uniform(-0.15, 0.15)
```

TensorBoard result:

```text
best checkpoint: 600k steps (also the final checkpoint)
50-episode mean reward: 1944.675
50-episode mean length: 1945.68 / 2000 steps = 19.46 s
peak rollout mean length: 1703.82 steps at 577,536
final rollout mean length: 1688.21 steps
```

This is a large improvement over run 9 for the intended hand-release balancing
task. The evaluation mean reached 97.3% of the maximum episode duration.

Current artifacts:

```text
RL/v1/trained_policies1/best10/best_model.zip
RL/v1/trained_policies1/best10/vecnorm.pkl
RL/v1/trained_policies1/ppo_balance10.zip
RL/v1/trained_policies1/vecnorm10.pkl
RL/v1/tb/PPO_10/
```

`play1.py` is configured for `RUN = 10` and `USE_BEST = True`.

## Current environment summary

```text
control rate:          100 Hz
physics rate:          500 Hz
episode limit:         20 seconds
fall threshold:        0.6 rad
action delay:          1–3 control steps (10–30 ms)
initial pitch:         +/-0.10 rad
initial pitch rate:    +/-0.15 rad/s
wheel sliding friction: 0.7–1.1
reward:                upright only
policy action:         one shared motor command
```

## Next work, in order

1. Evaluate `best10` on a fixed held-out seed set and record survival, pitch
   RMS, command-change RMS, saturation rate, and displacement. The TensorBoard
   result is excellent, but it is not yet the final robustness audit.
2. Inspect command chatter. Run 9's upright-only controller had high
   command-change RMS. If run 10 also chatters, an action-rate penalty has earned
   a controlled experiment; do not add it pre-emptively.
3. Reproduce the real encoder pipeline in simulation: encoder quantization,
   discrete differentiation, and the same low-pass filter used on hardware.
4. Measure IMU and command latency rather than relying only on white noise and
   broad delay randomization.
5. Validate pitch, gyro, encoder, and motor signs on tethered hardware before
   policy deployment. Use current limiting, a deadman, and a conservative real
   tilt cutoff.
6. Add relative wheel position to the observation and a position reward only
   after basic physical balancing works and measured floor drift requires it.
7. Expand initial pitch in measured curriculum stages (`0.10 -> 0.15 -> 0.20`
   rad) only if recovery robustness becomes a real requirement. Do not return to
   `+/-0.30` without evidence that those states are physically recoverable.

## Known cleanup items

- Some comments still say two actions or `2 * 3`; the arrays themselves use
  `self.num_actions` and are correct.
- The action-delay comment says 10–40 ms; the implemented range is 10–30 ms.
- `prev_action` remains initialized even though the action-rate reward is
  disabled.
- The one-element motor command currently reaches both actuators through NumPy
  broadcasting. It works, but explicitly assigning `[u, u]` would communicate
  the intent more clearly.
- All changes and run 9/10 artifacts are currently uncommitted in Git.

