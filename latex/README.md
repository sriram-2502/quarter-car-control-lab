# LaTeX course documents

- `project_handout.tex` builds the corrected student assignment at `../docs/project.pdf`.
- `project_solutions.tex` builds the detailed worked solution at `../solutions/solution-notes.pdf`.
- `preamble.tex` preserves the supplied document's one-column 10-point layout, numbered equations, boxed results, and colored MATLAB listing style. Standard `article` replaces the custom `ieeeconf` class, which was not included with the supplied sources; margins remain 0.75 inches.
- `figures/ME4030_project.png` is the original supplied system diagram.
- `figures/corrected_*.png` are regenerated directly in MATLAB using the original six-panel controller-plot layout.
- `generated/` contains reproducible tables and MATLAB numerical checks used by the builder.

The supplied `project_handout.tex.txt`, `figures/lqr_controller.png`, and `figures/pid_controller.png` are preserved as historical reference material. They contain older equations, tuning, and results and are **not** the corrected build targets.

From the repository root, use MATLAB with Control System Toolbox to run:

```matlab
run('tools/export_document_figures.m')
```

Then use a Python environment with NumPy and SciPy:

```powershell
python tools/build_documents.py
```

The builder runs Tectonic in Conda's `latex-env`, verifies the numerical tables against the model and the native MATLAB checks, and publishes both PDFs only after successful compilation. Override the Conda executable or environment with `--conda PATH --env NAME`. Tectonic may download LaTeX packages on the first build. No separate `pdflatex` installation is needed.

The solution includes the executable MATLAB files directly with `\lstinputlisting`; code edits therefore appear in the next PDF build. Parameters and documented example gains are checked before building so a later tuning change cannot silently leave stale derivations. Review changed equations and regenerate figures whenever tuning or the model changes.

The original assignment's 30/30/40 grading and 6/6/4 question structure are retained. Corrections cover force/output signs, the road-derivative-free coordinate, physical velocity reconstruction, ideal PID road feedthrough, integral LQR gains, zero-final-value overshoot, and accurate RMS integration. The original Fall 2025 dates are replaced by an undated public assignment with a corrected-edition date.
