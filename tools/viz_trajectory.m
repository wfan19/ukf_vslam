function viz_trajectory(r, q, options)

arguments
    r (:, 3) double
    q (:, 4) double
    options.a (:, 3) double
    options.frame_numb (1, 1) double = 15
    options.figure (1, 1) logical = 1
end

if options.figure
    figure()
end
hold on

%% Calculate indices for which TF frames to show
% Finds all factors of the total index number, and picks the factor closest
% to a desired number of frames to show. This way we ensure that the frames
% are always evenly distributed, at the cost of sometimes showing more/less
% than the desired number of frames.

n = length(r);
indices_all = 1:n;

% Calculate all factors for length of indices
all_factors = indices_all(rem(n, indices_all) == 0);

% Find index of factor closest to desired frame #
[~, i] = min(abs(all_factors - options.frame_numb));

% Create filtered indices for using nearest factor
indices_tform = linspace(0, n, all_factors(i)+1);
indices_tform(1) = 1;

%% Plotting
% Plot TF frames
plotTransforms(r(indices_tform, :), q(indices_tform, :))

% Plot accelerations at TF frames, if the argument is passed
if isfield(options, 'a')
    % Calculate accelerations in world frame
    a_world = rotatepoint(conj(quaternion(q(indices_tform, :))), options.a(indices_tform, :));
    % Quiver plot
    quiver3(r(indices_tform, 1), r(indices_tform, 2), r(indices_tform, 3),...
        a_world(:, 1), a_world(:, 2), a_world(:, 3)...
    )
end

% Plot trajectory path (at full resolution)
% We plot this last so that if you want to plot another path against it
% they will be back to back in the legends
plot3(r(:, 1), r(:, 2), r(:, 3))

xlabel("x")
ylabel("y")
zlabel("z")
title("Trajectory")

axis equal
grid on
fprintf("TF frames displayed: %d\n", all_factors(i)+1)

end