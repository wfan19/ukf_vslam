function tests = test_imu_predict
%tests = functiontests(localfunctions);
test_complex_path()
end

%{
%% Test if body states are updated correctly in the case of a purely linear motion
% TODO: Description of the test scenario
function testPureLinMvmtBodyStates(testCase)

end

%% Test if body states are updated correctly in the case of a purely rotational motion
% TODO: Description of the test scenario
function testPureRotMvmtBodyStates(testCase)

end

%% Test for pure
%}

%% Test arbitrary path
function test_complex_path(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Axis of rotation as a function of time
    alpha = [
        1;
        1;
        0;
    ];
    alpha = alpha / norm(alpha);
    % Angle of rotation as a function of time
    theta = 0.2*t;

    % Construct the quaternion using axis-angle
    q_t = [
        cos(theta/2);
        sin(theta/2)*alpha;
    ];

    % Path over time
    r_t = [
        sin(t);
        t;
        exp(0.3*t);
    ];

    %  Generate timespan vector
    n = 342;
    t_0 = 0;
    t_end = 10;

    tspan = linspace(t_0, t_end, n);

    sim_test(tspan, r_t, q_t);
end

%% Core IMU integration simulation
function sim_test(t, r_t, q_t)
    % Generate simulated dataset
    tab_sim = generate_trajectory(t, r_t, q_t);
    
    % Initialize state
    t = tab_sim.t(1);
    r_0 = tab_sim.r(1, :);
    v_0 = eval(subs(diff(r_t), struct('t', 0)));
    q_0 = tab_sim.q(1, :);
    
    tab_tags_0 = table();
    
    state_t = State(r_0, v_0, q_0, tab_tags_0);
    r_int = zeros(size(tab_sim.r));
    q_int = zeros(size(tab_sim.q));
    
    r_int(1, :) = r_0;
    q_int(1, :) = q_0;
    
    for i = 2 : length(tab_sim.t)
        dt = tab_sim.t(i) - t;
        t = tab_sim.t(i);
        input_t = ImuInput(tab_sim.omega(i-1, :), tab_sim.a(i-1, :));
        state_t = imu_predict(dt, state_t, input_t);
        
        r_int(i, :) = state_t.r_body';
        q_int(i, :) = state_t.q_body';
    end
    
    viz_trajectory(r_int, q_int)
end