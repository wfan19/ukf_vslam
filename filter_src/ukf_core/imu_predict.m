function state_next = imu_predict(dt, state, input)

state_next = state;

%% Construct current body state matrix
mat_state = eye(5);
mat_state(1:3, 1:3) = quat2rotm(state.q_body');
mat_state(1:3, 4) = state.r_body;
mat_state(1:3, 5) = state.v_body;

%% Preprocess input data
omega_hat = input.omega - state.b_omega;

%g_rotated = rotatepoint(conj(state.q_body), [0, 0, -9.8])'; % Gravity acceleration vector
g_rotated = rotatepoint(conj(quaternion(state.q_body')), [0, 0, 0])'; % Turns off gravity for testing
f_hat = input.f - state.b_f - g_rotated;

%% Construct twist matrix
mat_twist = zeros(5);
mat_twist(1:3, 1:3) = mat_skew_sym(omega_hat);
mat_twist(1:3, 4) = state.v_body;
mat_twist(1:3, 5) = f_hat;

%% Update state
mat_state_next = mat_state * expm(mat_twist*dt);

%% Extract output
state_next.q_body = rotm2quat(mat_state_next(1:3, 1:3))';
state_next.r_body = mat_state_next(1:3, 4);
state_next.v_body = mat_state_next(1:3, 5);
end

