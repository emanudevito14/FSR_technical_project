%% 1. SAMPLE TIME AND CRUISE VELOCITY
Ts = 0.004;                 % Sampling time (s)
v_cruise = 0.5;             % Target cruise speed (m/s)
w_yaw_max = deg2rad(20);    % Max allowed yaw rate (rad/s) for smooth orientation
%% 2. WAYPOINTS SETUP
waypoints = [
    0.0,  0.0,  1.5,  deg2rad(0);
    0.5,  0.5,  2.0,  deg2rad(10);
    1.0,  1.5,  2.0,  deg2rad(45);
    1.5,  1.5,  2.0,  deg2rad(30);
    2.75,  1.0,  2.0,  deg2rad(90)
];
N_wp = size(waypoints, 1);
N_seg = N_wp - 1;
%% 3. TIME ALLOCATION & CONTINUOUS VELOCITIES AND ACCELERATIONS
dists = zeros(1, N_seg);
T_seg = zeros(1, N_seg);
for i = 1:N_seg
    % Distance XYZ
    d_spatial = norm(waypoints(i+1, 1:3) - waypoints(i, 1:3));
    % Angular distance Yaw
    d_yaw = abs(waypoints(i+1, 4) - waypoints(i, 4));
    
    T_spatial = d_spatial / v_cruise;
    T_yaw     = d_yaw / w_yaw_max;
    
    T_seg(i)  = max([T_spatial, T_yaw, 1.5]); % Minimum 1.5s per segment
end

dot_wp = zeros(N_wp, 4);
for i = 2:(N_wp - 1)
    v_in  = (waypoints(i, :) - waypoints(i-1, :)) / T_seg(i-1);
    v_out = (waypoints(i+1, :) - waypoints(i, :)) / T_seg(i);
    dot_wp(i, :) = 0.5 * (v_in + v_out);
end
dot_wp(1, :)   = zeros(1, 4); 
dot_wp(end, :) = zeros(1, 4);

ddot_wp = zeros(N_wp, 4);
for i = 2:(N_wp - 1)
    a_in  = (dot_wp(i, :) - dot_wp(i-1, :)) / T_seg(i-1);
    a_out = (dot_wp(i+1, :) - dot_wp(i, :)) / T_seg(i);
    ddot_wp(i, :) = 0.5 * (a_in + a_out);
end
ddot_wp(1, :)   = zeros(1, 4);
ddot_wp(end, :) = zeros(1, 4);

%% 4. MULTI-SEGMENT TRAJECTORY GENERATION
p_tot = [];
dot_p_tot = [];
ddot_p_tot = [];
t_tot = [];
t_curr = 0;
for i = 1:N_seg
    p0 = waypoints(i, :);
    pf = waypoints(i+1, :);
    
    dot_p0 = dot_wp(i, :);
    dot_pf = dot_wp(i+1, :);
    
    ddot_p0 = ddot_wp(i, :);
    ddot_pf = ddot_wp(i+1, :);
    
    dddot_p0 = zeros(1, 4); 
    dddot_pf = zeros(1, 4);
    
    t0 = t_curr;
    tf = t_curr + T_seg(i);
    
    [p_seg, dot_p_seg, ddot_p_seg, t_seg] = segment(...
        p0, pf, dot_p0, dot_pf, ddot_p0, ddot_pf, dddot_p0, dddot_pf, t0, tf, Ts);
    
    if i == 1
        p_tot = p_seg;
        dot_p_tot = dot_p_seg;
        ddot_p_tot = ddot_p_seg;
        t_tot = t_seg;
    else
        p_tot = [p_tot, p_seg(:, 2:end)];
        dot_p_tot = [dot_p_tot, dot_p_seg(:, 2:end)];
        ddot_p_tot = [ddot_p_tot, ddot_p_seg(:, 2:end)];
        t_tot = [t_tot, t_seg(2:end)];
    end
    
    t_curr = tf;
end
%% 4.1 EXTENDED HOVERING PHASE AT FINAL WAYPOINT
T_hold = 5.0; % Extra seconds in steady hover
if T_hold > 0
    t_extra = (t_curr + Ts) : Ts : (t_curr + T_hold);
    N_extra = length(t_extra);
    
    p_extra = repmat(p_tot(:, end), 1, N_extra);
    dot_p_extra = zeros(4, N_extra);
    ddot_p_extra = zeros(4, N_extra);
    
    p_tot = [p_tot, p_extra];
    dot_p_tot = [dot_p_tot, dot_p_extra];
    ddot_p_tot = [ddot_p_tot, ddot_p_extra];
    t_tot = [t_tot, t_extra];
end
%% 5. SIMULINK PLANNER VARIABLES
t = t_tot;                           % Time vector (1 x N)
csi_d = p_tot(1:3, :);               % Position [x; y; z] (3 x N)
dot_csi_d = dot_p_tot(1:3, :);       % Velocity [vx; vy; vz] (3 x N)
ddot_csi_d = ddot_p_tot(1:3, :);     % Acceleration [ax; ay; az] (3 x N)
psi_d = p_tot(4, :);                 % Yaw angle (1 x N)
dot_psi_d = dot_p_tot(4, :);         % Yaw rate (1 x N)
ddot_psi_d = ddot_p_tot(4, :);       % Yaw acceleration (1 x N)
g = 9.81;
control_params;
m_sum_plant = m_q + m_L_plant;
%% 6. INITIAL CONDITIONS 
xl_0 = csi_d(:, 1);        
vl_0 = dot_csi_d(:, 1);    
e_x0 = xl_0 - csi_d(:, 1);
e_v0 = vl_0 - dot_csi_d(:, 1);
A_0 = -kx*e_x0 - kv*e_v0 + m_sum_plant * (ddot_csi_d(:, 1) + g*[0; 0; 1]);
norm_A0 = norm(A_0);
q_0 = -A_0 / norm_A0;
dot_A_0 = -kx * e_v0; 
dot_q_0 = -(dot_A_0 - (q_0' * dot_A_0) * q_0) / norm_A0; 
omega_0 = cross(q_0, dot_q_0);
l_0 = L + (m_L_plant * norm_A0) / (k_plant*m_sum_plant);
dot_l_0 = 0;
b3_0 = A_0 / norm_A0; 
b1_d = [cos(psi_d(1)); sin(psi_d(1)); 0]; 
b2_0 = cross(b3_0, b1_d) / norm(cross(b3_0, b1_d));
b1_0 = cross(b2_0, b3_0);
R_0  = [b1_0, b2_0, b3_0];
Omega_0 = [0; 0; 0];
%% 7. PLOTTING
% figure('Name', '3D Trajectory');
% plot3(csi_d(1, :), csi_d(2, :), csi_d(3, :), 'b-', 'LineWidth', 2); hold on;
% plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% grid on; axis equal;
% xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
% title('3D Generated Trajectory');
% legend('Trajectory', 'Waypoints');