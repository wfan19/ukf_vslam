function tests = test_imu_3d
tests = functiontests(localfunctions);
end

% Default settings for this test suite
function sim_imu(testCase, r_t, q_t)
    plotting = true;
    sim_imu_predict(testCase, r_t, q_t, 'plot', plotting);
end

%% Test 3d path (sinusoid in 1 dimension)
% 3d sin1: Sinusoid in x dimension, exponential in z dimension
function test_3d_sin1_rotated_no_rotation(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Construct the quaternion using axis-angle
    q = eul2quat([0, 0, pi/4], 'xyz')';

    % Path over time
    r_t = [
        sin(t);
        t;
        exp(0.3*t);
    ];

    sim_imu(testCase, r_t, q_t);
end

%% Test aribtrary path while rotating
% 3d sin1: Sinusoid in x dimension, exponential in z dimension
function test_3d_sin1_rotated_rotating(testCase)
    syms t
    assume(t, ["real", "positive"])

    % Axis of rotation as a function of time
    alpha = [
        1;
        0;
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

    sim_imu(testCase, r_t, q_t);
end
