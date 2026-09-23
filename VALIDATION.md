# Validation

Checked on 2026-09-23.

## Executed checks

- Algebraic coordinate transformation independently matches the physical two-mass force balance, including elimination of the road derivative.
- Actuator and road transfer functions match the independent mechanical derivation over 30 logarithmically spaced frequencies.
- Passive mechanical stability, four-state and augmented controllability, active closed-loop stability, LQR Riccati residual, and constant-road equilibria pass.
- Reconstructed PID force matches proportional, integral, and physical derivative feedback.
- All six passive/PID/LQR step/sine cases pass an independent adaptive Radau ODE integration with integrated output energies (relative energy tolerance 2e-6).
- Halving the output interval from 1 ms to 0.5 ms preserves sampled body positions and exact RMS energy integrals (relative energy tolerance 1e-7).
- MISS_HIT 0.9.44 parsed and linted all eight MATLAB files, including the native validation script, with no reported issues.
- All three PDFs were rendered and visually inspected. All six native MATLAB GIFs and six matching screenshots were checked for frame count, playback timing, file size, and visual correctness.

## Native MATLAB verification

Executed with MATLAB R2026b (26.2.0.3386108) and Control System Toolbox 26.2 on Windows. All six passive/PID/LQR step/sine cases passed native MATLAB execution and matched the published metrics. The maximum normalized metric difference was 4.51e-10 (tolerance 1e-6; normalization uses max(1, abs(reference))).

The shipped demo ran unchanged, producing its static plots and complete animation. All six animation paths, optional animation inputs, zero-motion inputs, and the intentional starter guard passed. Figures were rendered offscreen and exported for visual review; manual GUI interaction and PID Tuner tuning were not tested.

Native testing prompted fixes to the animation's fixed wheel/body offsets, endpoint margins, physical velocity display, final-frame handling, and figure colors/labels under the dark desktop theme. The controller equations and published numerical results did not change.

The reusable [MATLAB validator](tools/validate_matlab.m) runs from the repository to prevent older same-named files in another working directory from shadowing repository functions. The [native result report](solutions/matlab-validation.json) records the environment, case metrics, and checks.

## Model limits

All six checked-in GIFs are native MATLAB figure captures from `animate_quarter_car`; the six result figures are screenshots exported from the same animation. Each GIF contains 101 frames covering 10 s of motion at 10 fps plus a 1 s final hold. The Python companion only generates and checks numerical tables.

The reference is a linear, full-state, ideal-actuator model. The PID is ideal and unfiltered. Its original gains produce an approximately 7.81 MN peak at the discontinuous road step; this is explicitly reported, not presented as a practical design. The example controllers do not meet all instructional benchmarks.

RMS quantities use continuous interval integrals. Peak and recovery metrics use a 1 ms output grid; future excursions beyond the 10 s record are not assessed.

## Reproduce

From the repository root:

```sh
python -m pip install -r tools/requirements.txt
python tools/build_previews.py
python tools/validate_repository.py
```

Native MATLAB validation, starting from the repository root:

```matlab
addpath('tools');
report = validate_matlab;
export_matlab_media; % regenerate all six native GIFs and screenshots
```

`validate_matlab` writes its JSON report and figure exports to a temporary directory; an optional output-directory argument selects another location. `export_matlab_media` writes the published GIFs and screenshots into the repository.

Optional PDF rebuild and MATLAB static analysis:

```sh
python -m pip install reportlab miss_hit
python tools/build_documents.py
mh_lint solutions starter shared tools/validate_matlab.m tools/export_matlab_media.m
```

The student scaffold intentionally contains incomplete parameters, matrices, and tuning values. Its guards stop execution until the required tasks are completed.
