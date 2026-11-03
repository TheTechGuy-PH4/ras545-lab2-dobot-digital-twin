     clc; close all;

    j1 = 26.8640; j2 = -0.4890; j3 = 17.4398; j5 = 8.7759;

    theta1 = j1;
    theta2 = j2;
    theta3 = j3-j2;
    theta4 = -j3;
    theta5 = j5;

    % Convert degrees to radians:
    t1 = deg2rad(theta1);
    t2 = deg2rad(theta2);
    t3 = deg2rad(theta3);
    t4 = deg2rad(theta4);
    t5 = deg2rad(theta5);

    % DH parameters (as in your code)
    a1 = 0;  a2 = 150;  a3 = 150;  a4 = 90;  a5 = 0;

    % Compute forward kinematics once to get the desired (x, y, z)
    H05 = FK_5DOF(t1, t2, t3, t4, t5, a1, a2, a3, a4, a5);
    Px = H05(1,4); Py = H05(2,4); Pz = H05(3,4);
    desired_position = [Px, Py, Pz];
    fprintf('Forward Kinematics result (H05):\n');
    disp(H05);
    fprintf('Desired end-effector position (x,y,z) = [%.4f, %.4f, %.4f]',Px, Py, Pz);

    inv_kin_func = @(x) position_error_3vars(x, desired_position, a1, a2, a3, a4, a5);

    % Initial guess for [t1, t2, t5] in radians
    initial_guess = deg2rad([2.5, -1.0, 8.7759]);

    options = optimoptions('fsolve','Display','iter');
    [x_solution, fval] = fsolve(inv_kin_func, initial_guess, options);

    % x_solution = [t1, t2, t5]
    t1_sol = x_solution(1);
    t2_sol = x_solution(2);
    t5_sol = x_solution(3);
    t3_sol = deg2rad(j3) - t2_sol;
    t4_sol = -t3_sol;

    % Print out all angles in radians
    disp('Solved joint angles (in radians) [t1, t2, t3, t4, t5]:');
    disp([t1_sol, t2_sol, t3_sol, t4_sol, t5_sol]);

    % Print out all angles in degrees
    disp('Solved joint angles (in degrees) [t1, t2, t3, t4, t5]:');
    disp(rad2deg([t1_sol, t2_sol, t3_sol, t4_sol, t5_sol]));

    H05_sol = FK_5DOF(t1_sol, t2_sol, t3_sol, t4_sol, t5_sol, a1, a2, a3, a4, a5);
    final_xyz = H05_sol(1:3,4)';
    fprintf('\nFinal (x,y,z) from solved angles = [%.4f, %.4f, %.4f]', final_xyz);
    fprintf('Difference from desired = [%.4e, %.4e, %.4e]', final_xyz-desired_position);


%% ------------------------------------------------------------------------
function H05 = FK_5DOF(t1, t2, t3, t4, t5, a1, a2, a3, a4, a5)
    % Forward Kinematics for 5 DOF
    H01 = [ cos(t1),    0, -sin(t1), a1*cos(t1);
            sin(t1),    0,  cos(t1), a1*sin(t1);
                0,   -1,        0,           0;
                0,    0,        0,           1 ];


    H12 = [ cos(t2), -sin(t2), 0,  a2*sin(t2);
            sin(t2),  cos(t2), 0, -a2*cos(t2);
               0,         0,  1,           0;
               0,         0,  0,           1 ];


    H23 = [ cos(t3), -sin(t3), 0, a3*cos(t3);
            sin(t3),  cos(t3), 0, a3*sin(t3);
               0,         0,  1,          0;
               0,         0,  0,          1 ];


    H34 = [ cos(t4),  0, -sin(t4), a4*cos(t4);
            sin(t4),  0,  cos(t4), a4*sin(t4);
               0,   1,        0,           0;
               0,   0,        0,           1 ];

    H45 = [ cos(t5), -sin(t5), 0, a5*cos(t5);
            sin(t5),  cos(t5), 0, a5*sin(t5);
               0,         0,  1,         0;
               0,         0,  0,         1 ];


    H05 = H01 * H12 * H23 * H34 * H45;
end

%% ------------------------------------------------------------------------
function diff = position_error_3vars(x, desired_pos, a1, a2, a3, a4, a5)
    t1 = x(1);
    t2 = x(2);
    t5 = x(3);
    t3 = deg2rad(17.4398) - t2;
    t4 = -t3;

    % Compute forward kinematics for these angles
    H05_local = FK_5DOF(t1, t2, t3, t4, t5, a1, a2, a3, a4, a5);
    calculated_position = H05_local(1:3,4)';

    % Return the position error = [dx, dy, dz]
    diff = calculated_position - desired_pos;
end
