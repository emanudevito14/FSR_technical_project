%% 3D ANIMATION AND VIDEO SAVING SCRIPT [cite: Gemini AI]
% Extraction from Simulink Timeseries objects
t   = out.tout;                           % Time vector (N x 1)
x_d = squeeze(out.x_l_d.Data)';           % Nx3 matrix (Desired payload position)
x_L = squeeze(out.x_l.Data)';             % Nx3 matrix (Actual payload position)
q   = squeeze(out.q.Data)';               % Nx3 matrix (Cable orientation vector)
l   = squeeze(out.l.Data);                % Nx1 vector (Cable length)
R   = out.R.Data;                         % 3x3xN array (Quadrotor rotation matrix)

% Ensure vectors are oriented as N x 3
if size(x_d, 1) ~= length(t), x_d = x_d'; end
if size(x_L, 1) ~= length(t), x_L = x_L'; end
if size(q,   1) ~= length(t), q   = q';   end

% --- 1. RE-SAMPLING AT UNIFORM TIME STEPS FOR REAL-TIME PLAYBACK ---
fps = 30;
t_uniform = t(1) : (1/fps) : t(end);
N_unif = length(t_uniform);

% Interpolate 3D positions and orientation vectors
x_L_interp = interp1(t, x_L, t_uniform);
q_interp   = interp1(t, q,   t_uniform);
l_interp   = interp1(t, l,   t_uniform);

% Smooth interpolation of the 3x3xN rotation matrix
R_flat = reshape(R, 9, length(t))';
R_flat_interp = interp1(t, R_flat, t_uniform);
R_interp = reshape(R_flat_interp', 3, 3, N_unif);

% --- 2. FIGURE SETUP AND TARGET RESOLUTION ---
target_size = [600, 800]; % [Height, Width] in pixels (even numbers for H.264 codec)

fig = figure('Color', 'w', 'Name', '3D Quadrotor with Slung Load Simulation', ...
             'Position', [100, 100, 800, 600]);
grid on; hold on;

xlabel('X [m]', 'Interpreter', 'latex'); 
ylabel('Y [m]', 'Interpreter', 'latex'); 
zlabel('Z [m]', 'Interpreter', 'latex');
title('Quadrotor with payload suspended trajectory', 'Interpreter', 'latex');


view(-37.5, 30);

purpleColor = [0.5, 0, 0.5]; 

% Plot desired trajectory and waypoints
plot3(x_d(:,1), x_d(:,2), x_d(:,3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');
h_path = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Payload Trajectory');
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'o', ...
      'MarkerSize', 4, 'MarkerFaceColor', purpleColor, 'MarkerEdgeColor', purpleColor, ...
      'DisplayName', 'Waypoints');

% Graphical elements of the system
h_cable      = plot3(nan, nan, nan, 'k-', 'LineWidth', 2,   'DisplayName', 'Cable');          % Elastic cable
h_load       = plot3(nan, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Payload'); % Payload
h_uav_body   = plot3(nan, nan, nan, 'k-', 'LineWidth', 2.5, 'HandleVisibility', 'off');      % Quadrotor arms
h_uav_rotors = plot3(nan, nan, nan, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off'); % Rotors

d_drone = 0.3; % Quadrotor arm length (m)

% Force initial rendering to stabilize window dimensions
drawnow;

% --- 3. VIDEO INITIALIZATION AND ANIMATION LOOP ---
video_filename = 'uav_payload_simulation.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps; 
open(v);

for i = 1:N_unif
    % Interpolated current payload position, cable orientation, and length
    xl_i = x_L_interp(i, :)';
    q_i  = q_interp(i, :)';
    l_i  = l_interp(i);
    
    % Calculated Quadrotor position
    xq_i = xl_i - l_i * q_i;
    
    % Interpolated 3x3 rotation matrix at step i
    R_i = R_interp(:, :, i); 
    
    % Position of the 4 rotors in body frame (X-configuration)
    p1 = xq_i + R_i * [ d_drone;  0; 0];
    p2 = xq_i + R_i * [-d_drone;  0; 0];
    p3 = xq_i + R_i * [0;  d_drone; 0];
    p4 = xq_i + R_i * [0; -d_drone; 0];
    
    % Update traveled payload trajectory
    set(h_path, 'XData', x_L_interp(1:i, 1), 'YData', x_L_interp(1:i, 2), 'ZData', x_L_interp(1:i, 3));
    
    % Update cable (from xq to xl)
    set(h_cable, 'XData', [xq_i(1), xl_i(1)], 'YData', [xq_i(2), xl_i(2)], 'ZData', [xq_i(3), xl_i(3)]);
    
    % Update payload position
    set(h_load, 'XData', xl_i(1), 'YData', xl_i(2), 'ZData', xl_i(3));
    
    % Update UAV frame body
    set(h_uav_body, 'XData', [p1(1) p2(1) nan p3(1) p4(1)], ...
                    'YData', [p1(2) p2(2) nan p3(2) p4(2)], ...
                    'ZData', [p1(3) p2(3) nan p3(3) p4(3)]);
                
    % Update UAV rotors
    set(h_uav_rotors, 'XData', [p1(1) p2(1) p3(1) p4(1)], ...
                      'YData', [p1(2) p2(2) p3(2) p4(2)], ...
                      'ZData', [p1(3) p2(3) p3(3) p4(3)]);
    
    drawnow limitrate;
    
    % Capture frame and force resize to target_size [600, 800]
    frame = getframe(fig);
    frame.cdata = imresize(frame.cdata, target_size);
    
    writeVideo(v, frame);
end

close(v);
disp(['Video successfully saved: ', video_filename]);