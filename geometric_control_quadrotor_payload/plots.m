%% PLOTS  

%Extract signals from simulink

t_sim   = out.tout;                           % Time vector (N x 1)
x_d = squeeze(out.x_l_d.Data);           % Nx3 matrix (Desired payload position)
x_L = squeeze(out.x_l.Data);             % Nx3 matrix (Actual payload position)
err_x=-squeeze(out.ex.Data); %Nx3 matrix error of position load 
f=out.f.Data; %Nx1 vector thrust 
M=squeeze(out.M.Data); %Nx3 matrix torques input

tension=k_plant*(out.l.Data-L)+c_plant*out.dot_l.Data;

%% 1. 3D TRAJECTORY TRACKING PLOT

figure('Name', '3D Trajectory Tracking', 'Color', 'w');
h1 = plot3(x_d(:,1), x_d(:,2), x_d(:,3), '--r', 'LineWidth', 1.5); hold on;
h2 = plot3(x_L(:,1), x_L(:,2), x_L(:,3), 'b',   'LineWidth', 1.5);
hw = plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
grid on;

view(-37.5, 30); 

xlabel('$X$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('$Y$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
zlabel('$Z$ [m]', 'Interpreter', 'latex', 'FontSize', 11);

title('Payload 3D Trajectory Tracking', 'Interpreter', 'latex', 'FontSize', 12);

legend([h1, h2, hw], ...
       {'Desired Trajectory $\mathbf{p_L}_d$', 'Actual Trajectory $\mathbf{p}_L$', 'Waypoints'}, ...
       'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 10);

hold off;

%% 2. LOAD POSITION ERROR PLOT
figure('Name', 'Load Position Error', 'Color', 'w');
plot(t_sim, err_x(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, err_x(:,2), 'g', 'LineWidth', 1.5);
plot(t_sim, err_x(:,3), 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);

xlabel('Time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Position Error $\mathbf{e}_x$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Payload Position Error Components', 'Interpreter', 'latex', 'FontSize', 12);

legend({'$e_{p,x}$ ($x_d - x$)', '$e_{p,y}$ ($y_d - y$)', '$e_{p,z}$ ($z_d - z$)'}, ...
       'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 10);


%% 3. TOTAL CONTROL THRUST PLOT
figure('Name', 'Control Thrust', 'Color', 'w');
plot(t_sim, f,'m', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);

xlabel('Time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Thrust $f$ [N]', 'Interpreter', 'latex', 'FontSize', 11);
title('Total Control Thrust Input', 'Interpreter', 'latex', 'FontSize', 12);

legend({'Control Thrust $f$'}, 'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 10);

%% 4.  CONTROL TORQUES PLOT
figure('Name', 'Control Torques', 'Color', 'w');
plot(t_sim, M(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, M(:,2), 'g', 'LineWidth', 1.5);
plot(t_sim, M(:,3), 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]); 


xlabel('Time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Control Torques $\mathbf{M}$ [N$\cdot$m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Attitude Control Torques', 'Interpreter', 'latex', 'FontSize', 12);


legend({'$M_x$ (Roll)', '$M_y$ (Pitch)', '$M_z$ (Yaw)'}, ...
       'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 10);

%% Tension cable 
figure('Name', 'Tension cable', 'Color', 'w');
plot(t_sim, tension, 'r', 'LineWidth', 1.5); 
grid on;
xlim([0, t_sim(end)]); 


xlabel('Time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 11);
ylabel('Tension Cable $\mathbf{T}$ [N]', 'Interpreter', 'latex', 'FontSize', 11);
title('Tension Cable', 'Interpreter', 'latex', 'FontSize', 12);

