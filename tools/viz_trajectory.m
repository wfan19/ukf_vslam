function viz_trajectory(r, q, frame_numb)

% This is our ideal number of TF frames to display
% Defaults to 15
if ~exist('frame_numb')
    frame_numb = 15;
end

figure()
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
[~, i] = min(abs(all_factors - frame_numb));

% Create filtered indices for using nearest factor
indices_tform = linspace(0, n, all_factors(i)+1);
indices_tform(1) = 1;

%% Plotting
plotTransforms(r(indices_tform, :), q(indices_tform, :))
plot3(r(:, 1), r(:, 2), r(:, 3))

axis equal
grid on
fprintf("TF frames displayed: %d", all_factors(i)+1)

end