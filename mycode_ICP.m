%% Clear Environment
clear; clc; close all;

%% Load Point Cloud Data
X = readmatrix('pclX.txt');
Y = readmatrix('pclY.txt');

%% Initialize Transformation Parameters
R = eye(3); % Identity matrix for initial rotation
t = zeros(3, 1); % Zero vector for initial translation

%% Run ICP Algorithm
max_dist = 0.25;  % Maximum distance for point matching
max_iter = 30;    % Maximum ICP iterations
[R_opt, t_opt, matches] = ICP(X, Y, R, t, max_dist, max_iter);

%% Display Final Transformation
fprintf("Final Rotation Matrix:\n"); disp(R_opt);
fprintf("Final Translation Vector:\n"); disp(t_opt);

%% Apply Transformation to X
X_transformed = (R_opt * X' + t_opt)';

%% Compute Root Mean Square Error (RMSE)
diff = X_transformed(matches(:,1), :) - Y(matches(:,2), :);
rmse = sqrt(mean(sum(diff.^2, 2)));
fprintf("RMSE of aligned points: %g\n", rmse);

%% Plot Initial and Transformed Point Clouds
figure;
scatter3(X(:,1), X(:,2), X(:,3), '.b'); hold on;
scatter3(Y(:,1), Y(:,2), Y(:,3), '.r');
axis equal off; title('Original Point Clouds');

tiledlayout(1,2);
nexttile;
scatter3(X(:,1), X(:,2), X(:,3), '.b'); hold on;
scatter3(Y(:,1), Y(:,2), Y(:,3), '.r');
axis equal off; title('Original');
nexttile;
scatter3(X_transformed(:,1), X_transformed(:,2), X_transformed(:,3), '.b'); hold on;
scatter3(Y(:,1), Y(:,2), Y(:,3), '.r');
axis equal off; title('Aligned');

%% Compute Optimal Rigid Transformation
function [R, t] = best_rigid_transform(X, Y, matches)
    X_m = X(matches(:,1), :);
    Y_m = Y(matches(:,2), :);
    
    cX = mean(X_m, 1); cY = mean(Y_m, 1);
    X_c = X_m - cX; Y_c = Y_m - cY;
    
    [U, ~, V] = svd(Y_c' * X_c);
    R = U * diag([1,1,det(U*V')]) * V';
    t = cY' - R * cX';
end

%% Find Closest Points for Correspondences
function matches = estimate_correspondences(X, Y, R, t, max_dist)
    X_t = (R * X' + t)';
    num_X = size(X, 1);
    matches = zeros(num_X, 2);
    count = 0;
    
    for i = 1:num_X
        [min_dist, idx] = min(vecnorm(X_t(i,:) - Y, 2, 2));
        if min_dist <= max_dist
            count = count + 1;
            matches(count, :) = [i, idx];
        end
    end
    matches = matches(1:count, :);
end

%% Iterative Closest Point Algorithm
function [R, t, matches] = ICP(X, Y, R, t, max_dist, max_iter)
    for iter = 1:max_iter
        fprintf("ICP Iteration %d\n", iter);
        matches = estimate_correspondences(X, Y, R, t, max_dist);
        [R, t] = best_rigid_transform(X, Y, matches);
    end
    disp("ICP Converged!");
end
