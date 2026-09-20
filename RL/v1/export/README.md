# export/ — policy → ESP32 header + parity harness

Turns a trained v1 PPO balancer into C-consumable data, and **proves** the port is
numerically exact before any hardware is involved.

## The chain of trust

```
SB3 predict()  ==  np_forward()  ==  your C forward pass
  (the truth)     (proven here)    (you prove with fixtures.h)
```

- `export_policy.py` reimplements the exact inference path in numpy (`np_forward`) and
  checks it against SB3's own `predict()` on 500 real rollout obs. Agreement is ~2e-7.
  **If it doesn't match, it writes nothing** — a wrong export never produces a header.
- It then dumps the weights (`policy.h`) and test vectors (`fixtures.h`). Your C forward
  pass only has to match `np_forward`, and `fixtures.h` lets you check that offline over
  serial. Match to **~1e-4** (float32 rounding differs between numpy and the ESP32 FPU).

## Run it

```bash
# from RL/v1
python export/export_policy.py
```

Set `RUN` at the top of the script to pick the policy (`best{RUN}`). Output lands in
`export/run{RUN}/` so policies never collide:

```
export/run13/policy.h      # DATA ONLY: VecNorm affine + MLP weights
export/run13/fixtures.h    # 12 (raw_obs -> expected_action) pairs
```

Current deploy pick: **best13** (jitter 0.280, healthiest — see best-policies memory).

## What the C must implement (it's in policy.h's header comment too)

```
x = clip((obs - OBS_MEAN)/sqrt(OBS_VAR + POLICY_EPS), -POLICY_CLIP, POLICY_CLIP)
h = tanh(W0 @ x + B0)          // W0 is [out=64][in=19], so h[o] = sum_i W0[o][i]*x[i]
h = tanh(W1 @ h + B1)
a = WA @ h + BA                // action_net: NO activation
a = clip(a, -1, 1)             // SB3 clips the action to the Box bounds
```

- Weights are row-major `[out][in]` (torch's layout) → inner loop is `sum_i W[o][i]*x[i]`.
- `action_net` has **no** activation; the final `clip(a,-1,1)` is the action-space bound,
  and it's active in normal operation (actions saturate at ±1).
- Everything is float32.

## The live obs contract (for the bench side, NOT needed for the fixture test)

`fixtures.h` obs are pre-baked, so you can test the forward pass with zero sensors. When
you later assemble the 19-vector from real hardware it must match sim order/units/sign:

```
[ pitch, pitch_rate, wl, wr,        # core (newest)
  act_hist(3),                      # last 3 actions, front = newest
  obs_hist(3 blocks of core4) ]     # last 3 core-obs, front = newest
```

- `pitch` from BNO085 fused quat via the same atan2 convention as `get_pitch()`.
- `pitch_rate` = gyro-y. `wl`/`wr` = encoder-differentiated wheel velocities.
- Sanity: `OBS_MEAN` shows wheel-vel channels sit ~±2.2 (var ~17). If your live encoder
  velocities are on a wildly different scale, that's a unit/sign bug and the affine will
  amplify it.

## Assumptions (hold across all v1 runs; break these and re-check)

- Exactly 2 hidden **tanh** layers + separate continuous `action_net`.
- Policy trained with VecNormalize on obs; deterministic action = Gaussian mean, no
  squashing (no gSDE / `squash_output`).
- Change `net_arch`/activation/action type in a future train → update `extract()` and
  `np_forward()`. The parity check will fail loudly rather than ship a wrong header.

## Files

| file | what |
|---|---|
| `export_policy.py` | extract + parity check + emit headers |
| `run{RUN}/policy.h` | weights + norm stats (data only; you write the forward pass) |
| `run{RUN}/fixtures.h` | offline test vectors for the C |
