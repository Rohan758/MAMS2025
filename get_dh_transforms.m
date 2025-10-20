function [T_0_1, T_1_2] = get_dh_transforms(q, L)
    % Returns the transformation matrices based on DH parameters.
    % q: vector of joint angles [q1; q2]
    % L: vector of link lengths [L1; L2]

    % DH Parameter Table: [theta, d, a, alpha]
    % theta: rotation about Z
    % d: translation along Z
    % a: translation along X
    % alpha: rotation about X

    dh_params = [ q(1)   0   L(1)  -pi/2;  % Row for Link 1
                  q(2)   0   L(2)    0  ]; % Row for Link 2

    % Matrix for Link 1 (Frame 0 to 1)
    T_0_1 = dh_matrix(dh_params(1,:));

    % Matrix for Link 2 (Frame 1 to 2)
    T_1_2 = dh_matrix(dh_params(2,:));
end

function T = dh_matrix(dh_row)
    % Computes a single homogeneous transformation matrix from a DH row
    theta = dh_row(1);
    d = dh_row(2);
    a = dh_row(3);
    alpha = dh_row(4);

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0         ,  sin(alpha)           ,  cos(alpha)           , d           ;
         0         ,  0                    ,  0                    , 1           ];
end