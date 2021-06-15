function viz_trajectory(tab_traj, frame_numb)

% This is our ideal number of TF frames to display
% Defaults to 15
if ~exist('frame_numb', 'var')
    frame_numb = 15;
end

figure()
hold on

%% Calculate indices for which TF frames to show
% Finds all factors of the total index number, and picks the factor closest
% to a desired number of frames to show. This way we ensure that the frames
% are always evenly distributed, at the cost of sometimes showing more/less
% than the desired number of frames.

n = length(tab_traj.r);
indices_all = 1:n;

% Calculate all factors for length of indices
all_factors = indices_all(rem(n, indices_all) == 0);

% Find index of factor closest to desired frame #
[~, i] = min(abs(all_factors - frame_numb));

% Create filtered indices for using nearest factor
indices_tform = linspace(0, n, all_factors(i)+1);
indices_tform(1) = 1;

%% Plotting
plotTransforms(tab_traj.r(indices_tform, :), tab_traj.q(indices_tform, :))
plot3(tab_traj.r(:, 1), tab_traj.r(:, 2), tab_traj.r(:, 3))

% Plot accelerations at TF frames, if the field exists in the table
if any(strcmp('a', tab_traj.Properties.VariableNames))
    a_world = rotatepoint(conj(quaternion(tab_traj.q(indices_tform, :))), tab_traj.a(indices_tform, :));
    quiver3(tab_traj.r(...
        indices_tform, 1), tab_traj.r(indices_tform, 2), tab_traj.r(indices_tform, 3),...
        a_world(:, 1), a_world(:, 2), a_world(:, 3)...
    )
end

axis equal
grid on
fprintf("TF frames displayed: %d", all_factors(i)+1)

end