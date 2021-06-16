function tests = test_imu_predict
functions = localfunctions;
tests = functiontests(functions);
%}


end

%% Core IMU integration simulation
function sim_imu_predict(testCase, t, r_t, q_t, options)

    arguments
        testCase
        t
        r_t
        q_t
        options.plot (1, 1) logical = 0
    end

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
    
    function plot_func(failure)
        if xor(failure, options.plot)
            figure()
            subplot(1, 2, 1)
            viz_trajectory(tab_sim.r, tab_sim.q, 'figure', 0)
            plot3(r_int(:, 1), r_int(:, 2), r_int(:, 3));
            title("Reference trajectory")

            subplot(1, 2, 2)
            viz_trajectory(r_int, q_int, 'figure', 0)
            plot3(tab_sim.r(:, 1), tab_sim.r(:, 2), tab_sim.r(:, 3));
            title("Reconstructed trajectory")

            set(gcf, 'position', [250, 150, 1500 800])
        end
    end

    if options.plot
        plot_func(false)
    end
    
    r_rmse = sqrt(mean(tab_sim.r - r_int, 'all')^2);
    fprintf("Position RMSE: %fm\n", r_rmse);
    verifyLessThan(testCase, r_rmse, 0.5, @() plot_func(true));
end

%% ========================= Test Cases =========================
%% Test for straight line without rotation
function test_straight_no_rotation(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Constant orientation
    q_t = [
        1; 0; 0; 0
    ];

    % Path over time
    r_t = [
        t; 0 ; 0
    ];

    %  Generate timespan vector
    n = 342;
    t_0 = 0;
    t_end = 10;

    tspan = linspace(t_0, t_end, n);

    sim_imu_predict(testCase, tspan, r_t, q_t);
end

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

    sim_imu_predict(testCase, tspan, r_t, q_t);
end
