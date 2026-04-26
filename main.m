clear all
close
clc

%% Setup Paths

addpath(genpath('functions'));
addpath(genpath('models'));

%% Parameters

Ts = 0.05; % Sample time [s]
Ptime = 3; % Horizon time [s]
simulation_time = 20; % time of simulation [s]

pWB0 = [0 0 0]';
qWB0 = [1 0 0 0]';
vWB0 = zeros(3,1);
omegaB0 = zeros(3,1);
x0 = [pWB0; qWB0; vWB0; omegaB0]; % Initial state

N = ceil(Ptime/Ts); % Horizon steps

%% Trajectory planning

stationary_time = 4; % time of hovering [s]
R0 = 1; % radius of trajectory [m]
height = 2; % height of trajectory [m]
turns = 2; % # of turns

% trajectory computation: Spiral + Hovering
Xref_spiral = spiralTrajectory(simulation_time-stationary_time, Ts, R0, height, turns, pWB0, qWB0);
N_spiral = size(Xref_spiral, 2);
x_final = Xref_spiral(:, end);
N_hover = round(stationary_time / Ts);
Xref_hover = repmat(x_final, 1, N_hover);
Xref_hover(8:13, :) = 0;
Xref = [Xref_spiral, Xref_hover];

%% Plottings

figure; % Plot of the position
plot3(Xref(1,:), Xref(2,:), Xref(3,:), 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Position');
view(45,25);

figure; % Plot of the linear velocity
plot((1:size(Xref,2))*Ts, Xref(8:10,:)', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('v_x','v_y','v_z');
title('Linear Velocity');

figure; % Plot of the altitude profile
plot((1:size(Xref,2))*Ts, Xref(3,:), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Z [m]');
title('Altitude Profile');

q = Xref(4:7, :).'; % Plot of the Euler Angles
eul = quat2eul(q, "ZYX");
yaw   = unwrap(eul(:,1));
pitch = eul(:,2);
roll  = eul(:,3);
NN = size(eul,1);
samples = 0:Ts:simulation_time;
figure;
subplot(3,1,1);
plot(samples, yaw, 'LineWidth', 1.4);
ylabel('Yaw [rad]'); grid on;
subplot(3,1,2);
plot(samples, pitch, 'LineWidth', 1.4);
ylabel('Pitch [rad]'); grid on;
subplot(3,1,3);
plot(samples, roll, 'LineWidth', 1.4);
ylabel('Roll [rad]'); xlabel('Samples'); grid on;
sgtitle('Euler Angles');

angvel = Xref(11:13, :); % Plot of the Angular velocity
wx = angvel(1, :);
wy = angvel(2, :);
wz = angvel(3, :);
t = 0:Ts:simulation_time;
figure;
plot(t, wx, 'LineWidth',1.6); hold on;
plot(t, wy, 'LineWidth',1.6);
plot(t, wz, 'LineWidth',1.6);
grid on;
xlabel("Time [s]");
ylabel("Angular Velocity [rad/s]");
title("Angular Velocities");
legend("omega_x", "omega_y", "omega_z");

%% Simulation with Adaption

out_ad=sim('Project_Sim_Adapt');

figure % Plot of the inputs
stairs(t,out_ad.input_ad.signals.values(:,:),'Linewidth',2)
xlabel('t [s]')
ylabel('u_i(t) [N]')
xlim([0 simulation_time])
legend('u_1','u_2','u_3','u_4')
grid on

% Plot of the position error
err_val(:,:) = out_ad.err_ad.signals.values(:,1,:);
figure
plot(out_ad.err_ad.time,err_val(:,:),'Linewidth',2)
xlabel('t [s]')
ylabel('p(t) [m]')
xlim([0 simulation_time])
legend('p_x','p_y','p_z')
grid on

%% Simulation with Observer

out_ob=sim('Project_Sim_Obs');

figure % Plot of the inputs
stairs(t,out_ob.input_ob.signals.values(:,:),'Linewidth',2)
xlabel('t [s]')
ylabel('u_i(t) [N]')
xlim([0 simulation_time])
legend('u_1','u_2','u_3','u_4')
grid on

% Plot of the position error
err_val2(:,:) = out_ob.error_ob.signals.values(:,1,:);
figure
plot(out_ob.error_ob.time,err_val2(:,:),'Linewidth',2)
xlabel('t [s]')
ylabel('p(t) [m]')
xlim([0 simulation_time])
legend('p_x','p_y','p_z')
grid on