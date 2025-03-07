% Main script

% Given inputs
start_pos = [37.0, 3.0, 12.0];
start_ori = [-90.0, 0.0, -90.0];
end_pos = [1.0, 35.0, 12.0];
end_ori = [-90.0, 0.0, -90.0];
num_points = round(2.0 / 0.1);
trajectory = repmat(TrajectoryPoint(), num_points, 1);

% Generate the curved trajectory
GenerateCurvedTrajectory5DOF(start_pos, end_pos, start_ori, end_ori, trajectory, num_points);

% Extract the positions for plotting
x_points = arrayfun(@(tp) tp.position(1), trajectory);
y_points = arrayfun(@(tp) tp.position(2), trajectory);
z_points = arrayfun(@(tp) tp.position(3), trajectory);

% Plotting the curve
figure;
hold on;
plot3(x_points, y_points, z_points, '-o', 'DisplayName', 'Cubic Bézier Curve', 'Color', 'b');
scatter3([start_pos(1), end_pos(1)], [start_pos(2), end_pos(2)], [start_pos(3), end_pos(3)], 'r', 'filled', 'DisplayName', 'Start/End Points');
scatter3([x_points(1), x_points(end)], [y_points(1), y_points(end)], [z_points(1), z_points(end)], 'g', 'filled', 'DisplayName', 'Control Points');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Cubic Bézier Curve for Trajectory');
legend;
grid on;
hold off;

% Functions (CubicSpline, GenerateControlPoints, etc.) below this point:
