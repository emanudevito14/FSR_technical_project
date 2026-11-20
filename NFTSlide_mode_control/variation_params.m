%% PARAMETRIC UNCERTAINTY ANALYSIS - NFTSMC CONTROLLER
clear;
close all;
clc;
data;

% Scenario Matrix Definition [k (N/m), c (Ns/m), m_L (kg)]
scenarios = [
    % --- High Stiffness  ---
    1e5,  40, 0.5;  % Scenario 1:  High k, High c, Nominal m_L 
    1e5,  40, 1.05;  % Scenario 2:  High k, High c, Heavy m_L
    1e5,   15, 0.5;  % Scenario 3:  High k, Low c,  Nominal m_L
    1e5,   15, 1.05;  % Scenario 4:  High k, Low c,  Heavy m_L
    
    % --- Medium Stiffness ---
    6e4, 40, 0.5;  % Scenario 5:  Medium k, High c, Nominal m_L
    6e4, 40, 1.05;  % Scenario 6:  Medium k, High c, Heavy m_L
    6e4,  15, 0.5;  % Scenario 7:  Medium k, Low c,  Nominal m_L
    6e4,  15, 1.05;  % Scenario 8:  Medium k, Low c,  Heavy m_L
    
    % --- Low Stiffness ---
    3e4,  40, 0.5;  % Scenario 9:  Low k, High c, Nominal m_L
    3e4,  40, 1.05;  % Scenario 10: Low k, High c, Heavy m_L
    3e4,   15, 0.5;  % Scenario 11: Low k, Low c,  Nominal m_L
    3e4,   15, 1.05;  % Scenario 12: Low k, Low c,  Heavy m_L
];
num_scenarios = size(scenarios, 1);
sim_results   = cell(num_scenarios, 1);

%% 1. RUN ALL SIMULATIONS FOR NFTSMC CONTROLLER
for scen_idx = 1:num_scenarios
    k_plant   = scenarios(scen_idx, 1);
    c_plant   = scenarios(scen_idx, 2);
    m_plant = scenarios(scen_idx, 3);
        
    fprintf('Simulating Geometric Control Scenario %d/%d (k = %.0f N/m, c = %.0f Ns/m, m_L = %.2f kg)...\n', ...
            scen_idx, num_scenarios, k_plant, c_plant, m_plant);
    
    res = struct();
    res.status_success = true;
    
    try
        trajectory_smooth;
        res.T_expected = t_tot(end); 
        
        out = sim('slide_mode_control'); 
        
        res.t     = out.tout;
        p_q_sim   = out.p_q.Data;      
        alpha_sim = squeeze(out.alpha.Data); 
        beta_sim  = squeeze(out.beta.Data);    
        l_sim     = squeeze(out.l.Data);          
        U_sim     = squeeze(out.U.Data);
        
        if size(p_q_sim, 1) == 3 && size(p_q_sim, 2) ~= 3, p_q_sim = p_q_sim'; end
        if size(U_sim, 1) == 4 && size(U_sim, 2) ~= 4, U_sim = U_sim'; end
        
        x_l = p_q_sim(:,1) + l_sim .* sin(alpha_sim) .* cos(beta_sim);
        y_l = p_q_sim(:,2) + l_sim .* sin(beta_sim);
        z_l = p_q_sim(:,3) - l_sim .* cos(alpha_sim) .* cos(beta_sim);
        x_L = [x_l, y_l, z_l];
        
        x_d_interp = interp1(t_tot, csi_d(1,:), res.t);
        y_d_interp = interp1(t_tot, csi_d(2,:), res.t);
        z_d_interp = interp1(t_tot, csi_d(3,:), res.t);
        x_d_vec    = [x_d_interp(:), y_d_interp(:), z_d_interp(:)];
        
        res.err_x = x_L - x_d_vec;
        res.U     = U_sim;
        res.f     = U_sim(:, 1);     
        res.M     = U_sim(:, 2:4);   
        
    catch ME
        res.status_success = false;
        res.err_msg   = ME.message;
    end
    
    sim_results{scen_idx} = res;
end

%% 2. AUTOMATIC METRICS EXTRACTION (CON CHATTERING COPPIA)
fprintf('\n======================== METRICS EXTRACTION: NFTSMC ========================\n');
fprintf('Scen |    k    |  c |  m_L | Pos RMSE [m] | Max Thrust [N] | Max Torque [Nm] | Chat Torque [Nm] | Status\n');
fprintf('----------------------------------------------------------------------------------------------------\n');

for scen_idx = 1:num_scenarios
    res     = sim_results{scen_idx};
    k_val   = scenarios(scen_idx, 1);
    c_val   = scenarios(scen_idx, 2);
    m_L_val = scenarios(scen_idx, 3);
   
    is_failed   = false;
    fail_reason = '';
    
    if ~res.status_success
        is_failed   = true;
        fail_reason = 'SIM CRASH';
    elseif res.t(end) < (res.T_expected - 0.5)
        is_failed   = true;
        fail_reason = sprintf('EARLY STOP (%.1fs/%.1fs)', res.t(end), res.T_expected);
    elseif any(isnan(res.err_x(:))) || any(isnan(res.f(:))) || any(isnan(res.M(:)))
        is_failed   = true;
        fail_reason = 'NaN DETECTED';
    elseif any(isinf(res.err_x(:))) || any(isinf(res.f(:))) || any(isinf(res.M(:)))
        is_failed   = true;
        fail_reason = 'Inf DETECTED';
    elseif max(sqrt(sum(res.err_x.^2, 2))) > 2.0 % Criterio Divergence
        is_failed   = true;
        fail_reason = 'DIVERGED (>2m)';
    end
    
    if is_failed
        fprintf(' S%-2d | %6.0f | %2.0f | %3.1f |   %-10s |    %-10s  |    %-10s   |   %-14s | FAILED (%s)\n', ...
                scen_idx, k_val, c_val, m_L_val, 'N/A', 'N/A', 'N/A', 'N/A', fail_reason);
    else
        % 1. RMSE Error
        e_pos    = sqrt(sum(res.err_x.^2, 2)); 
        rmse_pos = sqrt(mean(e_pos.^2));
        
        % 2. max thrust value
        f_max    = max(abs(res.f));
        
        % 3. Max M_xy torque value
        torque_xy_norm = sqrt(res.M(:,1).^2 + res.M(:,2).^2);
        torque_max     = max(torque_xy_norm);
        
        % 4. Chattering 
        t_window  = 3.0; 
        idx_ss    = res.t >= (res.t(end) - t_window);
        torque_ss = torque_xy_norm(idx_ss);
        chat_Mxy  = peak2peak(detrend(torque_ss));
        
        if rmse_pos > 0.15
            status_str = 'Degraded';
        else
            status_str = 'Stable';
        end
        
        fprintf(' S%-2d | %6.0f | %2.0f | %4.2f |   %8.4f   |    %8.2f    |    %8.2f     |     %10.4f     | %s\n', ...
                scen_idx, k_val, c_val, m_L_val, rmse_pos, f_max, torque_max, chat_Mxy, status_str);
    end
end
fprintf('================================================================================--------------------\n');