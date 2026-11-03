# RAS 545 Lab 2 — Dobot Digital Twin and Inverse Kinematics

In this ASU Robotics 1 team lab, we built a Simulink digital twin of the Dobot Magician Lite and explored numerical inverse kinematics in MATLAB. The simulation model and standalone kinematics script are two separate parts of the project.

## Simulink digital twin

[`Assembly Files/`](Assembly%20Files/) contains the `Mag_lite_Assem_lab2.slx` model, its MATLAB parameter data file, the CAD import XML, and six STEP files for the robot link geometry. Keep these files together with their original names when opening the model in MATLAB with Simulink and Simscape Multibody.

## MATLAB kinematics example

[`Lab2RAS545.m`](Lab2RAS545.m) starts with a reference joint configuration and multiplies five homogeneous transforms to obtain the base-to-tool transform. It extracts the XYZ translation as the inverse-kinematics target.

The script uses `fsolve` to drive the difference between calculated and target XYZ toward zero. The residual function takes `t1`, `t2`, and `t5` as solver inputs and derives `t3` and `t4` from the linkage equations used in that function. After the solve, the script runs forward kinematics again and prints the joint angles, reconstructed XYZ, and component-wise position error.

## Run the example

Open MATLAB in the repository root with Optimization Toolbox available, then run:

```matlab
Lab2RAS545
```

The target comes from the script's own forward-kinematics calculation. The solve fits position only. With `a5 = 0`, `t5` does not change the XYZ residual, so the reported joint angles are not uniquely determined by that residual. The reference calculation and the solver also use different expressions for the fourth joint angle. Use the printed residual to assess the numerical fit for this example. Hardware accuracy and Simulink execution were not independently verified for this write-up.
