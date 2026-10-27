# Nonlinear control of a 2-DoF gimbal

Robust and adaptive Lyapunov-based controllers for a two-axis camera gimbal,
built as a project for a Master's course in Nonlinear Control Theory.

The write-up with the full derivation, the simulation results and the
controller comparison is here:
**[haydarkarhan.com/projects/nonlinear-control](https://haydarkarhan.com/projects/nonlinear-control/)**

The report submitted for the course is in [`docs/`](docs/).

## What is in here

The plant is a two-link gimbal — an azimuth joint carrying an elevation joint —
modelled as

```
M(q) qdd + C(q, qd) qd + G(q) = tau
```

written in terms of five lumped parameters so the dynamics are linear in the
unknowns, which is what makes the adaptive controller possible.

Five controllers are implemented, all built on the same filtered tracking error
`r = ed + alpha*e`:

| Controller | Idea | Stability |
| --- | --- | --- |
| Exact model knowledge | cancel the dynamics outright | uniform asymptotic |
| Robust, high gain | `VR = rho^2/eps * r` | exponential, then bounded |
| Robust, high frequency | `VR = rho^2/(rho*norm(r) + eps) * r` | uniformly ultimately bounded |
| Robust, sliding mode | `VR = rho*sign(r)` | exponential |
| Adaptive, standard | estimate theta online, `thetad = Gamma*Y'*r` | asymptotic (Barbalat) |
| Adaptive, desired model comp. | regressor along the reference only | asymptotic (Barbalat) |

The course report calls the desired-model-compensation case exponentially
stable. It is not: its Lyapunov function carries a parameter-error term
`0.5*thetatilde'*inv(Gamma)*thetatilde` that nothing in the bound drives down,
so `Vdot <= -lambda_min(Kr)*norm(r)^2` does not give `Vdot <= -c*V`. The `rho*r`
term buys robustness, not a stronger stability class. The robust rows above
really are exponential, because their Lyapunov function is `0.5*r'*M*r` alone.

Also inherited from the report: the d.m.c. implementation adds `rho*norm(r)`,
a scalar, to both channels, where the derivation calls for the vector `rho*r`.
The proof needs `r'*VR >= 0` and the implemented form goes negative about a
sixth of the time. It is 0.14% of the peak torque here, so nothing measurable
changes, but the code and the proof are not describing the same controller.

## Running it

Two independent ways to reproduce the results.

### 1. Plain MATLAB, no Simulink

`simulate_gimbal.m` integrates the closed loop directly. It needs base MATLAB
and nothing else, and it is what generated every figure in the write-up.

```matlab
result = simulate_gimbal("high gain", 'Uncertainty', true);
plot(result.t, result.e)
```

To reproduce the comparison table:

```matlab
compare_controllers
```

Controllers are `"emk"`, `"high gain"`, `"high frequency"`, `"sliding mode"`,
`"adaptive"` and `"dmc"`. Options are documented in `help simulate_gimbal`;
the useful ones are `Uncertainty` (perturb the plant so the controller's model
is wrong), `StopTime`, and `BoundaryLayer` for the sliding-mode variant.

**A note on the sliding-mode controller.** With the ideal `sign(r)` the
closed-loop time constant is `M22/Kr ≈ 4.3 us`, so an explicit fixed-step
integrator needs a step well below that before the answer means anything —
`simulate_gimbal` substeps to 10 us by default, which is *not* enough, and the
velocity error it reports for that case is dominated by integration error. Pass
a non-zero `BoundaryLayer` for the saturated version, which is well posed, is
what any real implementation would use, and is what the write-up quotes.

### 2. Simulink and Simscape Multibody

```matlab
START_HERE
```

This opens the app and the Simulink model. The app sets the controller, the
trajectory and the uncertainty; the model reads those through variant
subsystems, so **the app has to be running before the model will simulate**.

> **Warning.** Simulating the Simscape Multibody model opens Mechanics
> Explorer, which renders the 3-D animation on the GPU. On at least one laptop
> with switchable Intel/NVIDIA graphics this hung the display driver hard
> enough to take the whole machine down. If you only want the numbers, use
> `simulate_gimbal` above.

Requires Simulink, Simscape and Simscape Multibody.

## Files

```
simulate_gimbal.m          closed-loop simulation, no Simulink required
compare_controllers.m      reproduces the controller comparison table
gimbal_parameters.m        nominal physical and lumped parameters
gimbal_theta.m             lumped parameters, with optional uncertainty
gimbal_compute_dynamics.m  M, C and G at a given state
gimbal_dynamics.mlx        symbolic derivation of the dynamics
gimbal_simulink.slx        Simulink model with the Simscape Multibody plant
gimbal_app.mlapp           front end for the Simulink model
create_grid.m              floor extrusion data for the Simscape scene
step/                      CAD geometry referenced by the Multibody solids
hardware/                  Dynamixel servo connection helpers
docs/                      the course report
```

`hardware/` is included for reference only. The hardware loop was never
finished: it depends on a `dxlSimLibrary` Simulink library that is not part of
this repository, and the real system was never run.

## Unfinished controllers in the model

The app offers five top-level choices, but only three of them are finished.
Two of the five — `learning` and `other` — were started and not completed, and
their blocks still carry the labels of the methods they were going to be:

| Block in the model | What the label promises | What the code actually does |
| --- | --- | --- |
| `Controller selector/learning` | a learning controller | identical to the robust high-gain law |
| `other/Supertwisting` | super-twisting sliding mode | identical to the robust high-frequency law |
| `other/RISE` | RISE | partially written, see below |

`RISE` is the only one with original content. It builds a second-order filtered
error `r = edd + alpha*ed + beta*e` and computes `M`, `C`, `G` and the
derivatives `dM`, `dC`, `dG`, but the line that would assemble them into the
model term `N` is commented out, so not one of those six quantities reaches the
output. What the block returns is

```matlab
dtau = (Kr + eye(2))*r + gamma*sign(e);     % gamma = diag([1000 10000])
```

which is a sketch of the RISE structure rather than the method: the real law
integrates `sign` of the filtered error, not of the position error, and the
gains here were never tuned.

None of the three appear in the write-up or in `simulate_gimbal.m`, and nothing
in the results depends on them. They are left in the model because the wiring
and the variant selection are real work and a later attempt can build on them,
but do not mistake any of the three for a working implementation.

## Limitations

- No friction. The dominant unmodelled effect in a small geared servo is almost
  certainly larger than the parameter uncertainty these controllers are
  designed against.
- No actuator dynamics, torque limits or encoder quantization. The robust
  designs use an effective gain of about 110, which is only free because the
  simulation's torque source is ideal.
- The adaptive parameter estimates do not converge to the true values on a
  single-frequency reference, because it is not persistently exciting. Tracking
  still works; the estimates are not measurements.

## References

1. F. L. Lewis, D. M. Dawson and C. T. Abdallah, *Robot Manipulator Control:
   Theory and Practice*, 2nd ed. Marcel Dekker, 2004.
2. J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*. Prentice Hall, 1991.
3. H. K. Khalil, *Nonlinear Systems*, 3rd ed. Prentice Hall, 2002.
4. M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*.
   Wiley, 2005.
5. V. I. Utkin, *Sliding Modes in Control and Optimization*. Springer, 1992.
