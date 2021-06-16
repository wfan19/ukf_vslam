%% Core IMU integration simulation
function sim_imu_predict(testCase, r_t, q_t, options)
    arguments
        testCase
        r_t
        q_t
        options.tspan (1, :) double = default_tspan() % Timespan vector
        options.plot (1, 1) logical = false % Change for toggling default plotting behavior
        options.max_r_rmse (1, 1) double = 0.05 % Max RMSE (m). We are shooting for centimeter or even millimeter accuracy
    end
    
    % Print current test function name
    stack = dbstack;
    disp("==================================================================")
    fprintf("Test: %s\n", stack(2).name) % Print the function name of the calling function

    % Generate simulated dataset
    tab_sim = generate_trajectory(options.tspan, r_t, q_t);
    
    % Initialize state
    t = tab_sim.t(1);
    r_0 = tab_sim.r(1, :);
    v_0 = tab_sim.v(1, :);
    q_0 = tab_sim.q(1, :);
    
    tab_tags_0 = table();
    
    state_t = State(r_0, v_0, q_0, tab_tags_0);
    
    % Initialize logging matrices
    r_int = zeros(size(tab_sim.r));
    v_int = zeros(size(tab_sim.r));
    q_int = zeros(size(tab_sim.q));
    
    r_int(1, :) = r_0;
    v_int(1, :) = v_0;
    q_int(1, :) = q_0;
    
    % Simulate the predict function
    for i = 2 : length(tab_sim.t)
        dt = tab_sim.t(i) - t;
        t = tab_sim.t(i);
        input_t = ImuInput(tab_sim.omega(i-1, :), tab_sim.a(i-1, :));
        state_t = imu_predict(dt, state_t, input_t);
        
        r_int(i, :) = state_t.r_body';
        v_int(i, :) = state_t.v_body';
        q_int(i, :) = state_t.q_body';
    end
    
    % Plotting:
    % The <failure> param denotes whether the plot function was triggered
    % by a test failure or by request
    function plot_func(failure)
        % If plotting was requested *and* the test failed, we've already
        % plotted so we don't plot twice
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
    
    % Validate positional RMSE
    r_rmse = sqrt(mean(tab_sim.r - r_int, 1).^2);
    fprintf("Position RMSE: [%.4f, %.4f, %.4f]m\n", r_rmse(1), r_rmse(2), r_rmse(3));
    verifyLessThan(testCase, r_rmse, options.max_r_rmse, @() plot_func(true));
end

%% Default timespan function
function tspan = default_tspan()
    %  Generate timespan vector
    t_0 = 0;
    t_end = 10;
    hz = 100;
    n = (t_end - t_0) * hz;

    tspan = linspace(t_0, t_end, n);
end