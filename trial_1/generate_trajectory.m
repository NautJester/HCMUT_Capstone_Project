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

%% Function Definitions
function CubicSpline(start_pos, end_pos, control_pos1, control_pos2, trajectory, num_points)
    for j = 1:num_points
        t = (j - 1) / (num_points - 1);  % Normalized time
        t2 = t * t;
        t3 = t2 * t;

        for i = 1:3  % Loop for x, y, z coordinates
            trajectory(j).position(i) = ...
                (1 - t)^3 * start_pos(i) + ...
                3 * (1 - t)^2 * t * control_pos1(i) + ...
                3 * (1 - t) * t2 * control_pos2(i) + ...
                t3 * end_pos(i);
        end
    end
end

function [control_pos1, control_pos2] = GenerateControlPoints(start_pos, end_pos, offset_factor)
    direction = zeros(1, 3);
    length = 0.0;

    for i = 1:3
        direction(i) = end_pos(i) - start_pos(i);
        length = length + direction(i)^2;
    end

    length = sqrt(length);

    if length > 0
        direction = direction / length;
    end

    control_pos1 = start_pos + direction * offset_factor * length;
    control_pos2 = end_pos - direction * offset_factor * length;
end

function GenerateCurvedTrajectory5DOF(start_pos, end_pos, start_ori, end_ori, trajectory, num_points)
    offset_factor = 0.5;
    [control_pos1, control_pos2] = GenerateControlPoints(start_pos, end_pos, offset_factor);

    CubicSpline(start_pos, end_pos, control_pos1, control_pos2, trajectory, num_points);

    for j = 1:num_points
        t = (j - 1) / (num_points - 1);
        trajectory(j).orientation = InterpolateOrientation(start_ori, end_ori, t);
    end
end

function result_ori = InterpolateOrientation(start_ori, end_ori, t)
    result_ori = zeros(1, 3);
    result_ori(1) = start_ori(1) * (1 - t) + end_ori(1) * t;  % Roll
    result_ori(2) = 0.0;  % Pitch
    result_ori(3) = start_ori(3) * (1 - t) + end_ori(3) * t;  % Yaw
end
