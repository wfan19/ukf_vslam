function tests = test_imu_1d
tests = functiontests(localfunctions);
end

% Default settings for this test suite
function sim_imu(testCase, r_t, q_t)
    plotting = false;
    sim_imu_predict(testCase, r_t, q_t, 'plot', plotting);
end

%% Test for straight line without rotation
function test_straight_aligned_no_rotation(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Constant orientation
    q = [
        1; 0; 0; 0
    ];

    % Path over time
    r_t = [
        t; 0 ; 0
    ];

    sim_imu(testCase, r_t, q_t);
end

%% Test for travelling in a straight line with non-identity constant orientation
% The straight line is aligned along the x axis
function test_straight_rotated_no_rotation(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Constant orientation
    q_t = eul2quat([0, 0, pi/4], 'xyz')';
    
    % Path over time
    r_t = [
        t; 0 ; 0
    ];

    sim_imu(testCase, r_t, q_t);
end