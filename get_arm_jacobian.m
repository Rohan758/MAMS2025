function J = get_arm_jacobian(T_0_1, T_0_2)
    % Calculates the 6x2 geometric Jacobian for a 2-link arm.
    % T_0_1: Transform from base to link 1
    % T_0_2: Transform from base to link 2 (end-effector)

    % Position of the end-effector
    p_e = T_0_2(1:3, 4);

    % Z-axis of the base frame (axis of rotation for joint 1)
    z0 = [0; 0; 1];

    % Z-axis of the link 1 frame (axis of rotation for joint 2)
    R_0_1 = T_0_1(1:3, 1:3);
    z1 = R_0_1 * [0; 0; 1];

    % --- Jacobian Columns ---
    % Column 1 (for joint 1)
    Jv1 = cross(z0, p_e);
    Jw1 = z0;
    J1 = [Jv1; Jw1];

    % Column 2 (for joint 2)
    p1 = T_0_1(1:3, 4); % Position of joint 2
    Jv2 = cross(z1, p_e - p1);
    Jw2 = z1;
    J2 = [Jv2; Jw2];

    J = [J1, J2];
end