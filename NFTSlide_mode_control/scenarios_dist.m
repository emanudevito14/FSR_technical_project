%% EXTERNAL DISTURBANCE REJECTION ANALYSIS - NFTSMC CONTROLLER (TABLE METRICS ONLY)
clear;
close all;
clc;
data;

coeff = [0.5; 1.0; 1.5; 2.0; 2.5; 3.0; 5.0; 10.0];
num_scenarios = length(coeff);
sim_results   = cell(num_scenarios, 1);

%% 1. RUN ALL SIMULATIONS FOR NFTSMC CONTROLLER UNDER DISTURBANCES
for scen_idx = 1:num_scenarios
    
    disturbances; 
    
    K_dist = coeff(scen_idx);
    
    Awind_0     = K_dist * Awind_0;
    A_wind      = K_dist * A_wind;
    tau_phi_0   = K_dist * tau_phi_0;
    tau_theta_0 = K_dist * tau_theta_0;
    tau_psi_0   = K_dist * tau_psi_0;
    A_tau       = K_dist * A_tau;
    
    fprintf('Simulating NFTSMC Disturbance Scenario %d/%d (K_dist = %.1f)...\n', ...
            scen_idx, num_scenarios, K_dist);
    
    res = struct();
    res.status_success = true;
    res.K_dist    = K_dist;
    res.Awind_0   = Awind_0;
    res.tau_0     = tau_phi_0; 
    
    try
        
        trajectory_smooth;
        res.T_expected = t_tot(end); 
        
        out = sim('slide_mode_control_with_pertu'); 
        
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

%% 2. AUTOMATIC METRICS EXTRACTION (METRICHE MASSIME + CHATTERING COPPIA)
fprintf('\n======================== METRICS EXTRACTION: NFTSMC ========================\n');
fprintf('Scen | K_dist | Pos RMSE [m] | Max Thrust [N] | Max Torque [Nm] | Chat Torque [Nm] | Status\n');
fprintf('-----------------------------------------------------------------------------------------\n');

for scen_idx = 1:num_scenarios
    res    = sim_results{scen_idx};
    K_dist = res.K_dist;
    
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
        fprintf(' D%-2d | %6.1f |   %-10s |    %-10s  |    %-10s   |   %-14s | FAILED (%s)\n', ...
                scen_idx, K_dist, 'N/A', 'N/A', 'N/A', 'N/A', fail_reason);
    else
        % 1. RMSE Error
        e_pos    = sqrt(sum(res.err_x.^2, 2)); 
        rmse_pos = sqrt(mean(e_pos.^2));
        
        % 2. max thrust value
        f_max    = max(abs(res.f));
        
        % 3. Max M_xy torque value
        torque_xy_norm = sqrt(res.M(:,1).^2 + res.M(:,2).^2);
        torque_max     = max(torque_xy_norm);
        
        % 4. chattering
        t_window  = 3.0; 
        idx_ss    = res.t >= (res.t(end) - t_window);
        torque_ss = torque_xy_norm(idx_ss);
        chat_Mxy  = peak2peak(detrend(torque_ss));
        
        if rmse_pos > 0.15
            status_str = 'Degraded';
        else
            status_str = 'Stable';
        end
        
        fprintf(' D%-2d | %6.1f |   %8.4f   |    %8.2f    |    %8.2f     |     %10.4f     | %s\n', ...
                scen_idx, K_dist, rmse_pos, f_max, torque_max, chat_Mxy, status_str);
    end
end
fprintf('================================================================================---------\n');