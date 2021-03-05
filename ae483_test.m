%% Clear workspace
clear
clc
close all

%% Find equations of motion
[h, x, u, h_num, params] = ae483_01_findeoms();

%% Do control design
[K, x_e, u_e] = ae483_02_docontroldesign(h, x, u, h_num, params);

%% Simulate
[data] = ae483_03_simulate(h_num, K, x_e, u_e);

%% Visualize

% Parse data from simulation
t = data.t; % time, s
o = data.x(1:3, :); % position vector, m
hy = data.x(4, :); % yaw, rad
hp = data.x(5, :); % pitch, rad
hr = data.x(6, :); % roll, rad
v = data.x(7:9, :); % velocity vector, m/s
w = data.x(10:12, :); % angular velocity vector, rad/s
o(3,901)


moviefile = [];     % <--- could give the name of a file to save a movie
                    %           of the visualization to, like 'test.mp4'
ae483_visualize(t, o, hy, hp, hr, moviefile); 

% 
% figure; hold on; box on; grid on;
% plot3(o(1,:),o(2,:),o(3,:),'r'); % plot quadrotor trajectory
% plot3(o(1,1:5:end),o(2,1:5:end), o(3,1:5:end),'b.','markersize',10); % plot points along trajectory equally spaced in time
% xlabel('X, m');
% ylabel('Y, m');
% zlabel('Z, m');
% view(3)

figure(2)
subplot(2,2,1)
plot(t,o(1,:),'r')
yline(0,'-.b')
axis([0 inf -3 3])
title('X position vs time')

subplot(2,2,2)
plot(t,o(2,:),'r')
yline(0,'-.b')
axis([0 inf -3 3])
title('Y position vs time')

subplot(2,2,3)
yline(1,'-.b')
plot(t,o(3,:)*-1,'r')

title('Z position vs time')


o(3,901)


% y = transpose(o(3,:))
% sz = size(y);
% z=ones(sz);
% size(z)
% R = corrcoef(y,z)

%z




% C = [1 0 0 0 0 0 0 0 0 0 0 0;0 1 0 0 0 0 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0 0 0 0;0 0 0 1 0 0 0 0 0 0 0 0;0 0 0 0 1 0 0 0 0 0 0 0;0 0 0 0 0 1 0 0 0 0 0 0;0 0 0 0 0 0 1 0 0 0 0 0;0 0 0 0 0 0 0 1 0 0 0 0;0 0 0 0 0 0 0 0 1 0 0 0;0 0 0 0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 0 0 0 1];
% y = C*x
% a1 = laplace(u)
% a2 = laplace(y)
% a1/a2
