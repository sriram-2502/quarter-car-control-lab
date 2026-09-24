"""Build the corrected handout and worked solutions with Conda/Tectonic.

Run with a Python environment containing numpy/scipy. LaTeX itself runs in
``latex-env``; use --conda and --env to select another Conda installation.
"""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import tempfile
import numpy as np
import build_previews as model

ROOT = Path(__file__).resolve().parents[1]


def make_tables():
    """Check stored metrics against the model before typesetting their values."""
    model.validate_model()
    saved = json.loads((ROOT / 'solutions/results.json').read_text())
    native = json.loads((ROOT / 'latex/generated/matlab-document-checks.json').read_text())
    # The prose derives this specific example; fail instead of publishing stale math.
    expected = dict(Ms=2500, Mu=320, Ks=80000, Cs=350, Kt=500000,
                    Ct=15020, Kp=208000, Kd=832000, Ki=624000, R=0.01,
                    Q=[3e7, 8e7, 8e7, 2e7, 2e10], dt=0.001, duration=10,
                    step_amplitude=0.2, step_time=1, sine_amplitude=0.1,
                    sine_frequency=0.5)
    for key, value in expected.items():
        np.testing.assert_allclose(model.p[key], value, err_msg=f'Update document derivation for {key}')
    rows = [
        (r'$a_{\mathrm{RMS}}$ (m/s$^2$)', 'acceleration_rms_m_s2', '.3f'),
        (r'$x_{h,\mathrm{RMS}}$ (m)', 'tire_deflection_rms_m', '.5f'),
        (r'$\max|x_s|$ (m)', 'max_suspension_deflection_m', '.5f'),
        (r'$u_{\mathrm{RMS}}$ (kN)', 'force_rms_kN', '.3f'),
        (r'$u_{\mathrm{peak}}$ (kN)', 'force_peak_kN', '.3f'),
    ]
    out = []
    for road in ('step', 'sine'):
        rises = []
        for mode in ('NONE', 'PID', 'LQR'):
            run = model.simulate(mode, road)
            current = model.metrics(run)
            for key, value in current.items():
                if isinstance(value, (int, float)):
                    np.testing.assert_allclose(saved[road][mode][key], value, rtol=1e-8, atol=1e-9)
                    np.testing.assert_allclose(native[road][mode]['metrics'][key], value, rtol=1e-8, atol=1e-9)
                else:
                    assert saved[road][mode][key] == value
            if road == 'step':
                crossings = [np.flatnonzero((run['t'] >= model.p['step_time']) &
                             (run['zs'] >= level * model.p['step_amplitude']))
                             for level in (0.1, 0.9)]
                rise = run['t'][crossings[1][0]] - run['t'][crossings[0][0]]
                np.testing.assert_allclose(rise, native[road][mode]['body_rise_time_s'], atol=1e-10)
                rises.append(f'{rise:.3f}')
        out += [r'\subsection*{' + ('Step road' if road == 'step' else 'Sinusoidal road') + '}',
                r'\begin{center}\begin{tabular}{lrrr}\toprule',
                r'Metric & Passive & PID & LQR\\\midrule']
        active_rows = rows + ([
            (r'Body overshoot (\%)', 'body_overshoot_percent', '.2f'),
            ('Body settling (s)', 'body_settling_time_s', '.3f'),
            ('Suspension recovery (s)', 'suspension_recovery_time_s', '.3f'),
        ] if road == 'step' else [])
        for label, key, spec in active_rows:
            values = [saved[road][mode][key] for mode in ('NONE', 'PID', 'LQR')]
            strings = [format(value, spec) if isinstance(value, (int, float)) else str(value)
                       for value in values]
            out.append(label + ' & ' + ' & '.join(strings) + r'\\')
        if rises:
            out.append(r'Body rise, 10--90\% (s) & ' + ' & '.join(rises) + r'\\')
        out.append(r'\bottomrule\end{tabular}\end{center}')
    (ROOT / 'latex/generated/results_tables.tex').write_text('\n'.join(out) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conda', default=os.environ.get('CONDA_EXE') or shutil.which('conda') or
                        str(Path.home() / 'miniconda3/Scripts/conda.exe'))
    parser.add_argument('--env', default='latex-env')
    args = parser.parse_args()
    make_tables()
    targets = [('project_handout', ROOT / 'docs/project.pdf'),
               ('project_solutions', ROOT / 'solutions/solution-notes.pdf')]
    with tempfile.TemporaryDirectory(prefix='quarter-car-latex-') as temp:
        for name, target in targets:
            command = [args.conda, 'run', '--no-capture-output', '-n', args.env,
                       'tectonic', '--keep-logs', '--outdir', temp, name + '.tex']
            result = subprocess.run(command, cwd=ROOT / 'latex', text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            print(result.stdout)
            if result.returncode:
                raise RuntimeError(f'LaTeX failed for {name}')
            log = (Path(temp) / (name + '.log')).read_text(errors='replace')
            problems = ('Overfull \\hbox', 'Overfull \\vbox', 'There were undefined references',
                        'LaTeX Warning: Reference', 'Missing character:')
            if any(problem in log for problem in problems):
                print('\n'.join(line for line in log.splitlines()
                                if any(problem in line for problem in problems)))
                raise RuntimeError(f'Fix typesetting warnings in {name} before publishing')
        # Publish both only after both compile successfully.
        for name, target in targets:
            shutil.copyfile(Path(temp) / (name + '.pdf'), target)
            print(f'Built {target.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
