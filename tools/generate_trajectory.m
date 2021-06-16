function tab_out = generate_trajectory(tspan, r_t, q_t)
% Generates a trajectory table with position, acceleration, ang-vel, etc
% info given parametric position and orientations, and a list of selection
% times
% - tspan: A time vector of 
% - r_t: 3d symbolic position vector as a function of parameter t
% - q_t: 4d symbolic quaternion vector as a function of parameter t
%
% Outputs:
% tab_out: Table containing the following information, at timestamps
% specified by tspan
% - r: Position
% - a: Acceleration
% - q: Orientation
% - omega: Angular rates

%% Derive symbolic groupwise velocity
% g_circ = g_inv*gdot, mapping from world velocity to groupwise velocity by
% inverting the current group state as a transformation
% The key concept is extrinsic transformations. See section 2.3.7 in 
% Geometric Mechanics, Hatton&Choset

% Calculate omega(t) and v(t) using a first order tangent space
g_t = construct_pose(q_t, r_t);
g_dot = simplify(diff(g_t));
g_inv = simplify(construct_pose(invert_quat(q_t), -r_t));
g_circ_right = simplify(g_inv*g_dot);

% Extract omega(t) and v(t) from g_circ, the twist matrix
omega_t = [
    g_circ_right(2, 1);
    -g_circ_right(3, 1);
    g_circ_right(3, 2)
];
v_t_body = g_circ_right(1:3, 4);

% Calculate a(t) in the body frame by applying the extrinsic transformation
% to the world-acceleration instead of the world-velocity
v_t_world = g_dot(1:3, 4);
h_t = g_t;
h_t(1:3, 4) = v_t_world;
h_inv = g_inv;
h_inv(1:3, 4) = v_t_world;

h_dot = simplify(diff(h_t));
h_circ_right = simplify(h_inv*h_dot);

% Extract a(t) from the extrinsi
a_t_body = h_circ_right(1:3, 4);

%% Substitution
struct_vals = struct('t', tspan);
f_sub = @(expression)(eval(subs(expression, struct_vals)));

r = f_sub(r_t)';
q = f_sub(q_t)';
v = f_sub(v_t_body)';
a = f_sub(a_t_body)';
omega = f_sub(omega_t)';

t = tspan(:);
tab_out = table(t, r, v, a, q, omega);

%% Functions
    % Quaternion vector to rotation matrix in homogenous coordinates
    function mat = quat_vec_to_mat(q)
        % Reurnts the corresponding rotation matrix for a quaterion q, in
        % homogenous coordinates
        [w, x, y, z] = deal(q(1), q(2), q(3), q(4));

        % Formula from http://groups.csail.mit.edu/graphics/classes/6.837/F01/Lecture09/Slide24.html
        mat = [
           1-2*y^2-2*z^2, 2*x*y-2*w*z,   2*x*z+2*w*y,   0;
           2*x*y+2*w*z,   1-2*x^2-2*z^2, 2*y*z-2*w*x,   0;
           2*x*z-2*w*y,   2*y*z+2*w*x,   1-2*x^2-2*y^2, 0;
           0,             0,             0,             1;
        ];
    end

    % Construct SE(3) pose matrix out of a quaternion and a position vector
    function pose_out = construct_pose(q, r)
        pose_out = sym(quat_vec_to_mat(q));
        pose_out(1:3, 4) = r;
    end

    % Invert quaternion
    % The inverse of a quaternion's rotation is just the complex-conjugate
    % of the quaternion
    function q_out = invert_quat(q)
        q_out = q;
        q_out(2:4) = -q(2:4);
    end

end

