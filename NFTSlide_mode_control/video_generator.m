%% 3D ANIMATION AND VIDEO SAVING SCRIPT [cite: Gemini AI]
% Extraction from Simulink Timeseries objects
t_vid     = out.tout;                % Time vector (variable step)
p_q_vid   = out.p_q.Data;            % Quadrotor position (N x 3)
alpha_vid = out.alpha.Data;          % Swing angle alpha
beta_vid  = out.beta.Data;           % Swing angle beta
l_vid     = out.l.Data;              % Cable length
R_vid     = out.R.Data;              % 3x3xN array (Quadrotor rotation matrix)

% Direct computation of Payload Position (x_L) using Alpha, Beta, L
x_L_vid = zeros(length(t_vid), 3);
x_L_vid(:,1) = p_q_vid(:,1) + l_vid .* sin(alpha_vid) .* cos(beta_vid);
x_L_vid(:,2) = p_q_vid(:,2) + l_vid .* sin(beta_vid);
x_L_vid(:,3) = p_q_vid(:,3) - l_vid .* cos(alpha_vid) .* cos(beta_vid);


fps = 30;
t_uniform = t_vid(1) : (1/fps) : t_vid(end);
N_unif = length(t_uniform);


p_q_interp = interp1(t_vid, p_q_vid, t_uniform);
x_L_interp = interp1(t_vid, x_L_vid, t_uniform);


R_flat = reshape(R_vid, 9, length(t_vid))';
R_flat_interp = interp1(t_vid, R_flat, t_uniform);
R_interp = reshape(R_flat_interp', 3, 3, N_unif);

target_size = [600, 800]; 

fig = figure('Color', 'w', 'Name', '3D Quadrotor with Slung Load Simulation', ...
             'Position', [100, 100, 800, 600]);
grid on; hold on;

xlabel('X [m]', 'Interpreter', 'latex'); 
ylabel('Y [m]', 'Interpreter', 'latex'); 
zlabel('Z [m]', 'Interpreter', 'latex');
title('3D Animation - Quadrotor \& Slung Load (NFTSMC)', 'Interpreter', 'latex');

view(-37.5, 30);

purpleColor = [0.5, 0, 0.5];

% Plot desired trajectory directly from planner data
plot3(csi_d(1,:), csi_d(2,:), csi_d(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');

plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'o', ...
      'MarkerSize', 4, 'MarkerFaceColor', purpleColor, 'MarkerEdgeColor', purpleColor, ...
      'DisplayName', 'Waypoints');

% Initialize graphical handles for the animation
h_path       = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Payload Trajectory');
h_cable      = plot3(nan, nan, nan, 'k-', 'LineWidth', 2,   'DisplayName', 'Cable');          
h_load       = plot3(nan, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Payload'); 
h_uav_body   = plot3(nan, nan, nan, 'k-', 'LineWidth', 2.5, 'HandleVisibility', 'off');      
h_uav_rotors = plot3(nan, nan, nan, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off'); 

d_drone = 0.3; % Quadrotor arm length (m)

drawnow;

% Video initialization
video_filename = 'uav_payload_simulation_ru.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps; 
open(v);

for i = 1:N_unif
    % Position of quadrotor and payload at step i
    xq_i = p_q_interp(i, :)';
    xl_i = x_L_interp(i, :)';
    
    % Extract 3x3 rotation matrix R at step i
    R_i = R_interp(:, :, i); 
    
    % Position of rotors in body frame rotated by R
    p1 = xq_i + R_i * [ d_drone;  0; 0];
    p2 = xq_i + R_i * [-d_drone;  0; 0];
    p3 = xq_i + R_i * [0;  d_drone; 0];
    p4 = xq_i + R_i * [0; -d_drone; 0];
    
    % Update payload path
    set(h_path, 'XData', x_L_interp(1:i, 1), 'YData', x_L_interp(1:i, 2), 'ZData', x_L_interp(1:i, 3));
    
    % Update cable directly from quadrotor xq to payload xl
    set(h_cable, 'XData', [xq_i(1), xl_i(1)], ...
                 'YData', [xq_i(2), xl_i(2)], ...
                 'ZData', [xq_i(3), xl_i(3)]);
             
    % Update payload dot
    set(h_load, 'XData', xl_i(1), 'YData', xl_i(2), 'ZData', xl_i(3));
    
    % Update UAV body and rotors with correct orientation R
    set(h_uav_body, 'XData', [p1(1) p2(1) nan p3(1) p4(1)], ...
                    'YData', [p1(2) p2(2) nan p3(2) p4(2)], ...
                    'ZData', [p1(3) p2(3) nan p3(3) p4(3)]);
                
    set(h_uav_rotors, 'XData', [p1(1) p2(1) p3(1) p4(1)], ...
                      'YData', [p1(2) p2(2) p3(2) p4(2)], ...
                      'ZData', [p1(3) p2(3) p3(3) p4(3)]);
    
    drawnow limitrate;
    
    frame = getframe(fig);
    frame.cdata = imresize(frame.cdata, target_size);
    
    writeVideo(v, frame);
end

close(v);
disp(['Video successfully saved: ', video_filename]);