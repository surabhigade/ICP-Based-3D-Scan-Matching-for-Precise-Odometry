%% Clear environment
clear; clc; close all;

%% Load point cloud data
pointCloud = textread('pclX.txt');

%% Process the point cloud
% Rearrange columns for desired order
rearrangedCloud = pointCloud(:, [3, 1, 2]);

% Downsample the point cloud
indices = 1:4:size(pointCloud, 1);
X = rearrangedCloud(indices, :);

% Total points in downsampled cloud
numPoints = size(X, 1);

%% Display the original point cloud
scatter3(X(:, 1), X(:, 2), X(:, 3), '.');
axis equal off; grid off;

%% Define rigid transformation y = R*x + t
skewMatrix = 2 * [0, -0.1, -0.12;
                  0.1,  0,  0.15;
                  0.12, -0.15, 0];

disp("Applied transformation: ");
R = expm(skewMatrix);
t = [0.5; -0.3; 0.3];

%% Transform point cloud and add noise
Y_true = (R * X' + t)';  
randomOrder = randperm(numPoints);
Y = Y_true(randomOrder, :) + 0.005 * randn(size(X));

%% Plot original and transformed point clouds
figure();
scatter3(X(:, 1), X(:, 2), X(:, 3), '.b'); hold on;
scatter3(Y(:, 1), Y(:, 2), Y(:, 3), '.r');
axis equal off; grid off;

%% Save processed point clouds
writematrix(X, 'pclX.txt', 'Delimiter', ' ');
writematrix(Y, 'pclY.txt', 'Delimiter', ' ');
