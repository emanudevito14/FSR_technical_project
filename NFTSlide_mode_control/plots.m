%% PLOTS 

t_sim = out.tout;
p_q_sim = out.p_q.Data;      
alpha_sim = out.alpha.Data; 
beta_sim = out.beta.Data;    
l_sim = out.l.Data;          
U_sim = out.U.Data;         
dot_l_sim=out.dot_l.Data;

x_l = p_q_sim(:,1) + l_sim .* sin(alpha_sim) .* cos(beta_sim);
y_l = p_q_sim(:,2) + l_sim .* sin(beta_sim);
z_l = p_q_sim(:,3) - l_sim .* cos(alpha_sim) .* cos(beta_sim);


x_l_d_interp = interp1(t_tot, csi_d(1,:), t_sim);
y_l_d_interp = interp1(t_tot, csi_d(2,:), t_sim);
z_l_d_interp = interp1(t_tot, csi_d(3,:), t_sim);


e_x = x_l_d_interp(:) - x_l;
e_y =  y_l_d_interp(:)-y_l;
e_z = z_l_d_interp(:)-z_l ;

Tension=k*(l_sim-L_rest)+c*dot_l_sim;

% --- Plot 1: 3D Tracking Trajectory ---
figure('Name', 'Tracking Trajectory');
plot3(x_l_d_interp, y_l_d_interp, z_l_d_interp, 'r--', 'LineWidth', 1.5); hold on;
plot3(x_l, y_l, z_l, 'b-', 'LineWidth', 1.5);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g');

grid on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend("Load Desired Trajectory", "Load Real Trajectory","Waypoints");
title('3D Tracking Trajectory of the Suspended Load');
view(3);

% --- Plot 2: Load Position Error ---
figure('Name', 'Load Position Error');
plot(t_sim, e_x, 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, e_y, 'g', 'LineWidth', 1.5);
plot(t_sim, e_z, 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);
xlabel('Time [s]'); ylabel('Error [m]');
legend('e_x', 'e_y', 'e_z');
title('Load Position Error');

% --- Plot 3: Thrust Control Input (U1) ---
figure('Name', 'Thrust Input U1');
plot(t_sim, U_sim(:,1), 'k', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);
xlabel('Time [s]'); ylabel('Thrust f');
legend('f (Thrust)');
title('Thrust Control Input');

% --- Plot 4: Torque Control Inputs (U2, U3, U4) ---
figure('Name', 'Torque Inputs U2, U3, U4');
plot(t_sim, U_sim(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, U_sim(:,3), 'g', 'LineWidth', 1.5);
plot(t_sim, U_sim(:,4), 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);
xlabel('Time [s]'); ylabel('Torques M_x, M_y, M_z');
legend('M_x (Roll)', 'M_y (Pitch)', 'M_z (Yaw)');
title('Torque Control Inputs');

% --- Plot 5: Swing Angles (Alpha and Beta) ---
figure('Name', 'Swing Angles');
plot(t_sim, alpha_sim, 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, beta_sim, 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);
xlabel('Time [s]'); ylabel('Angle [rad]');
legend('\alpha', '\beta');
title('Load Swing Angles');

%--- Plot 6: Tension cable ---
figure('Name', 'Tension cable');
plot(t_sim, Tension, 'b', 'LineWidth', 1.5);
grid on;
xlim([0, t_sim(end)]);
xlabel('Time [s]'); ylabel('Tension [N]');
title('Tension cable');