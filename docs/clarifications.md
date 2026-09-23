# Model and metric clarifications

The repository documents are corrected editions of the Fall 2025 ME 4030 materials. The original files in Box are unchanged.

1. **Tire forces:** the wheel equation uses `-Kt*(zu-zr) - Ct*(zu_dot-zr_dot)`. Both forces oppose displacement or velocity relative to the road. Some original equations had the opposite signs; the supplied transformed matrices already corresponded to the restoring signs.
2. **Output and transfer functions:** use `s = zs-zu` consistently. The actuator channel is `S/U`, and the road channel is `S/Zr`. The original task descriptions and displayed denominators were interchanged. Reversing the output sign also reverses each transfer function and the feedback convention.
3. **State definition:** the fourth state is `eta = s_dot - (Ct/Mu)*zs + alpha*s + (Ct/Mu)*zr`, where `alpha = Cs/Ms + Cs/Mu + Ct/Mu`. It is not `s_dot`, nor the input-only integral described in parts of the original derivation. The corrected model PDF derives all four equations directly.
4. **PID feedback:** the derivative term acts on physical `s_dot`. In transformed coordinates it includes a road-dependent term. The corrected simulator and starter reconstruct this term explicitly.
5. **Road disturbance:** the step is 0.2 m at 1 s, and the sine is 0.1 m at 0.5 Hz. All comparisons use 10 s records.
6. **Transient metrics:** suspension deflection returns to zero, so percentage overshoot relative to its final value is undefined. Report peak absolute suspension deflection and recovery to an absolute +/-0.004 m band (2% of road-step height). This band is a clarified teaching convention, not a vehicle standard. Body overshoot relative to 0.2 m and body 2% settling time are separately labeled supplemental metrics. Times start at disturbance onset. No step transient metrics are assigned to sine inputs.
7. **RMS integration:** exact interval integrals of squared acceleration, tire deflection, and actuator force avoid the error from coarse trapezoids across sharp PID transients. Continuous sine inputs are propagated without a staircase approximation.
8. **Example gains:** the provided PID gains and LQR weights are illustrative course settings, not a claim that every target is achievable or met. The original ideal PID gains produce a large force peak under a discontinuous road step; the corrected results show that rather than hiding it.
9. **Animation:** drawings are schematic, with fixed offsets. Per-frame wheel position clipping has been removed from the MATLAB animator so it does not suppress simulated motion.

The corrected assignment asks students to distinguish achieved targets from unmet ones and explain tuning tradeoffs. It retains the original grading weights (30/30/40) and the non-transient performance benchmarks as instructional benchmarks.
