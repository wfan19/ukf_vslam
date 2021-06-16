function tests = test_imu_2d
tests = functiontests(localfunctions);
end

% Default settings for this test suite
function sim_imu(testCase, r_t, q_t)
    plotting = true;
    sim_imu_predict(testCase, r_t, q_t, 'plot', plotting);
end

%% Test flat sinusoid, no rotation
function test_flat_sinusoid_no_rotation(testCase)
    syms t
    assume(t, ["real", "positive"])
    
    % Construct the quaternion using axis-angle
    q_t = eul2quat([0, 0, 0], 'xyz')';

    % Path over time
    r_t = [
        t;
        sin(t);
        0;
    ];

    sim_imu(testCase, r_t, q_t);
end