%% EXTERNAL DISTURBANCE REJECTION ANALYSIS - GEOMETRIC CONTROLLER (TABLE METRICS ONLY)
clear;
close all;
clc;
data;

coeff = [0.5; 1.0; 1.5; 2.0; 2.5; 3.0; 5.0; 10.0];
num_scenarios = length(coeff);
sim_results   = cell(num_scenarios, 1);

%% 1. RUN ALL SIMULATIONS FOR GEOMETRIC CONTROLLER UNDER DISTURBANCES
for scen_idx = 1:num_scenarios
    disturbances; 

    K_dist = coeff(scen_idx);
    
    Awind_0     = K_dist * Awind_0;
    A_wind      = K_dist * A_wind;
    tau_phi_0   = K_dist * tau_phi_0;
    tau_theta_0 = K_dist * tau_theta_0;
    tau_psi_0   = K_dist * tau_psi_0;
    A_tau       = K_dist * A_tau;
    
    fprintf('Simulating Geometric Disturbance Scenario %d/%d (K_dist = %.1f)...\n', ...
            scen_idx, num_scenarios, K_dist);
    
    res = struct();
    res.status_success = true;
    res.K_dist    = K_dist;
    res.Awind_0   = Awind_0;
    res.tau_0     = tau_phi_0; 
    
    try
      
        trajectory_smooth;
        res.T_expected = t_tot(end); 
        
        out = sim('geometric_control_pertub'); 
        
        res.t     = out.tout;                         
        res.err_x = squeeze(out.ex.Data); 
        res.f     = out.f.Data; 
        res.M     = squeeze(out.M.Data);
        
       
        if size(res.err_x, 1) == 3 && size(res.err_x, 2) ~= 3, res.err_x = res.err_x'; end
        if size(res.M, 1) == 3 && size(res.M, 2) ~= 3, res.M = res.M'; end
        
    catch ME
        res.status_success = false;
        res.err_msg   = ME.message;
    end
    
    sim_results{scen_idx} = res;
end

%% 2. AUTOMATIC METRICS EXTRACTION 
fprintf('\n================ METRICS EXTRACTION: DISTURBANCE REJECTION (GEOMETRIC) ================\n');
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
    elseif max(sqrt(sum(res.err_x.^2, 2))) > 2.0 
        is_failed   = true;
        fail_reason = 'DIVERGED (>2m)';
    end
    
  
    if is_failed
        fprintf(' D%-2d | %6.1f |   %-10s |    %-10s  |    %-10s   |   %-14s | FAILED (%s)\n', ...
                scen_idx, K_dist, 'N/A', 'N/A', 'N/A', 'N/A', fail_reason);
    else
      
        e_pos    = sqrt(sum(res.err_x.^2, 2)); 
        rmse_pos = sqrt(mean(e_pos.^2));
       
        f_max    = max(abs(res.f));
       
        torque_xy_norm = sqrt(res.M(:,1).^2 + res.M(:,2).^2);
        torque_max     = max(torque_xy_norm);
      
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