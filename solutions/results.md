# Reference comparison

Generated from `config.json`; 10 s records; exact continuous step/sine propagation with 1 ms output samples and exact interval RMS integration. These are illustrative gains, not an optimized design.

Body overshoot and body settling are supplemental metrics. Suspension recovery uses an absolute ±0.004 m band (2% of the 0.2 m road step). Percentage overshoot about zero suspension deflection is undefined. See [clarifications](../docs/clarifications.md).

## Step

| Metric | Passive | PID | LQR |
|---|---:|---:|---:|
| acceleration rms m s2 | 2.498 | 13.28 | 0.7532 |
| tire deflection rms m | 0.01364 | 0.01845 | 0.00613 |
| max suspension deflection m | 0.2207 | 0.006251 | 0.196 |
| force rms kN | 0 | 33.17 | 3.05 |
| force peak kN | 0 | 7810 | 33.31 |
| body overshoot percent | 95.35 | 54.31 | 3.205 |
| body settling time s | not settled | 1.27 | 1.621 |
| suspension recovery time s | not settled | 0.142 | 1.618 |

## Sine

| Metric | Passive | PID | LQR |
|---|---:|---:|---:|
| acceleration rms m s2 | 1.593 | 0.8597 | 0.4499 |
| tire deflection rms m | 0.008302 | 0.004743 | 0.002419 |
| max suspension deflection m | 0.1176 | 0.001286 | 0.1044 |
| force rms kN | 0 | 2.141 | 5.384 |
| force peak kN | 0 | 8.15 | 7.88 |
| body overshoot percent | N/A | N/A | N/A |
| body settling time s | N/A | N/A | N/A |
| suspension recovery time s | N/A | N/A | N/A |
