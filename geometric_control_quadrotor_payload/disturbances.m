%% Parameters external process disturbances(Wind, downwash, drag,damping)

%wind parameters
omg_w  = 0.8;   
A_wind =0.1;   
Awind_0    = 0.5; 

kdw = 0.05; %rotor downwash coefficient
%viscous damping coefficients
          %eta =[x,y,z,alpha,beta]
Cv_eta = [0.3; 0.3; 0.15; 0.4; 0.4]; 
         % gamma=[phi,theta,psi]
Cv_gamma = [0.05; 0.05; 0.02];

%areodynamic drag coefficients
Cd_eta = [0.08; 0.08; 0.05; 0.1; 0.1];
Cd_gamma = [0.01; 0.01; 0.01];

%zero-order torque disturbances
tau_phi_0   = 0.01;   
tau_theta_0 = 0.01;  
tau_psi_0   = 0.005;  
A_tau = 0.002;        