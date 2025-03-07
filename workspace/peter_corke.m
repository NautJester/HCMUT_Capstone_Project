run('path_to_toolbox/startup_rvc.m');
% Thông số DH parameters
d1 = 14.4; % khoảng cách d1
a2 = 10.5; % độ dài a2
a3 = 13;   % độ dài a3
d5 = 18;   % khoảng cách d5

% Khởi tạo DH parameters theo bảng
L1 = Link('d', d1, 'a', 0, 'alpha', -pi/2, 'revolute');
L2 = Link('d', 0, 'a', a2, 'alpha', 0, 'revolute');
L3 = Link('d', 0, 'a', a3, 'alpha', 0, 'revolute');
L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'revolute');
L5 = Link('d', d5, 'a', 0, 'alpha', 0, 'revolute');

% Tạo robot từ các links
robot = SerialLink([L1 L2 L3 L4 L5], 'name', '5DOF Robot');

% Hiển thị robot trong trạng thái mặc định
figure;
robot.teach();

% Khởi tạo các giá trị góc khớp để quét workspace
q1 = linspace(0, pi, 20); % Khớp 1: từ 0 đến 360 độ
q2 = linspace(0, pi/2, 20); % Khớp 2: từ -90 đến 90 độ
q3 = linspace(0, pi/2, 20); % Khớp 3: từ -90 đến 90 độ
q4 = linspace(0, pi, 20); % Khớp 4: từ 0 đến 180 độ
q5 = linspace(-pi, pi, 20); % Khớp 5: từ -180 đến 180 độ

% Tạo workspace
workspace = [];
for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            for l = 1:length(q4)
                for m = 1:length(q5)
                    % Tính toán vị trí cuối (end-effector)
                    q = [q1(i), q2(j), q3(k), q4(l), q5(m)];
                    T = robot.fkine(q);
                    workspace = [workspace; T.t']; % Lưu vị trí (x, y, z)
                end
            end
        end
    end
end

% Vẽ workspace
figure;
scatter3(workspace(:,1), workspace(:,2), workspace(:,3), '.');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace của robot 5DOF');
grid on;
