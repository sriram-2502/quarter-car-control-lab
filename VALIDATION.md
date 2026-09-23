# Validation

Checked on 2026-09-23.

## Executed checks

- Algebraic coordinate transformation independently matches the physical two-mass force balance, including elimination of the road derivative.
- Actuator and road transfer functions match the independent mechanical derivation over 30 logarithmically spaced frequencies.
- Passive mechanical stability, four-state and augmented controllability, active closed-loop stability, LQR Riccati residual, and constant-road equilibria pass.
- Reconstructed PID force matches proportional, integral, and physical derivative feedback.
- All six passive/PID/LQR step/sine cases pass an independent adaptive Radau ODE integration with integrated output energies (relative energy tolerance 2e-6).
- Halving the output interval from 1 ms to 0.5 ms preserves sampled body positions and exact RMS energy integrals (relative energy tolerance 1e-7).
- MISS_HIT 0.9.44 parsed and linted all five MATLAB files with no reported issues.
- All three PDFs were rendered and visually inspected. Both animations and the comparison plots were visually reviewed.

## Execution limits

MATLAB was not installed in this environment. Native MATLAB execution and interactive graphics have **not** been verified. Numerical results and GIFs were produced by the Python companion, whose equations and configuration match the MATLAB implementation. A MATLAB syntax check is not a substitute for running MATLAB.

The reference is a linear, full-state, ideal-actuator model. The PID is ideal and unfiltered. Its original gains produce an approximately 7.81 MN peak at the discontinuous road step; this is explicitly reported, not presented as a practical design. The example controllers do not meet all instructional benchmarks.

RMS quantities use continuous interval integrals. Peak and recovery metrics use a 1 ms output grid; future excursions beyond the 10 s record are not assessed.

## Reproduce

From the repository root:

```sh
python -m pip install -r tools/requirements.txt
python tools/build_previews.py
python tools/validate_repository.py
```

Optional PDF rebuild and MATLAB static analysis:

```sh
python -m pip install reportlab miss_hit
python tools/build_documents.py
mh_lint solutions starter shared
```

The student scaffold intentionally contains incomplete parameters, matrices, and tuning values. Its guards stop execution until the required tasks are completed.
