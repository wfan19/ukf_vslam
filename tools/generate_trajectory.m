function tab_out = generate_trajectory(tspan, r_t, q_t)

g_t = construct_pose(q_t, r_t);
g_dot = simplify(diff(g_t));
g_inv = simplify(construct_pose(invert_quat(q_t), -r_t));
g_circ_right = simplify(g_inv*g_dot);

omega_t = [
    g_circ_right(2, 1);
    -g_circ_right(3, 1);
    g_circ_right(3, 2)
];
v_t_body = g_circ_right(1:3, 4);
a_t_body = simplify(diff(v_t_body));

struct_vals = struct('t', tspan);
f_sub = @(expression)(eval(subs(expression, struct_vals)));

r = f_sub(r_t)';
q = f_sub(q_t)';
a = f_sub(a_t_body)';
omega = f_sub(omega_t)';

tab_out = table(tspan(:), r, q, a, omega);

    function mat = quat_vec_to_mat(q)
        % Reurnts the corresponding rotation matrix for a quaterion q, in
        % homogenous coordinates
        [w, x, y, z] = deal(q(1), q(2), q(3), q(4));

        % Formula from http://groups.csail.mit.edu/graphics/classes/6.837/F01/Lecture09/Slide24.html
        mat = [
           1-2*y^2-2*z^2, 2*x*y+2*w*z,   2*x*z-2*w*y,   0;
           2*x*y-2*w*z,   1-2*x^2-2*z^2, 2*y*z+2*w*x,   0;
           2*x*z+2*w*y,   2*y*z-2*w*x,   1-2*x^2-2*y^2, 0;
           0,             0,             0,             1;
        ];
    end

    function pose_out = construct_pose(q, r)
        pose_out = sym(quat_vec_to_mat(q));
        pose_out(1:3, 4) = r;
    end

    function q_out = invert_quat(q)
        q_out = q;
        q_out(2:4) = -q(2:4);
    end

end

