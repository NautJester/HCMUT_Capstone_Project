% Parameters
a2 = 10.5;  % Replace with ik->a2
a3 = 13.0;  % Replace with ik->a3
a4 = 2.5;   % Replace with ik->a4
d1 = 5.5;   % Replace with ik->d1
d5 = 10.0;  % Replace with ik->d5

% Discretization steps
theta1_steps = linspace(0, pi, 10);
theta2_steps = linspace(-pi/2, pi/2, 10);
theta3_steps = linspace(-pi/2, pi/2, 10);
R0_5_22_steps = linspace(-1, 1, 10);

% Initialize arrays to store x, y, z values
x_values = [];
y_values = [];
z_values = [];

for theta1 = theta1_steps
    for theta2 = theta2_steps
        for theta3 = theta3_steps
            for R0_5_22 = R0_5_22_steps
                % Calculate x, y, z based on the given code logic
                r = a2 * cos(theta2) + a3 * cos(theta2 + theta3);
                s = a2 * sin(theta2) + a3 * sin(theta2 + theta3) + d1;
                
                x = r * cos(theta1) + (a4 + d5);
                y = r * sin(theta1) + (a4 + d5);
                z = s + (a4 + d5) * R0_5_22;

                % Store the results
                x_values(end+1) = x;
                y_values(end+1) = y;
                z_values(end+1) = z;
            end
        end
    end
end

% Plotting the workspace
figure;
scatter3(x_values, y_values, z_values, 1, 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace of the Robot');
grid on;

% Display range of x, y, z
x_range = [min(x_values) max(x_values)];
y_range = [min(y_values) max(y_values)];
z_range = [min(z_values) max(z_values)];

disp(['X Range: ', num2str(x_range)]);
disp(['Y Range: ', num2str(y_range)]);
disp(['Z Range: ', num2str(z_range)]);
