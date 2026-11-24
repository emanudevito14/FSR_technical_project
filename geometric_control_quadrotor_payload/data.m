%% data model
m_q= 2;  % quadrotor mass
Ixx=0.0211; %kg*m^2
Iyy=0.0211; %kg*m^2
Izz = 0.0366; %kg*m^2
J=  diag([Ixx, Iyy, Izz]);    %Inertia matrix of the quadrotor with respect to body frame
m_L=0.5;% Load mass nominal known from controller
m_L_plant = m_L; %Real load mass of system plant
L= 1;%free length of  spring
epsilon=1; %scale factor
k_plant=1e5/epsilon^2;%Real load mass of system plant
c_plant=40/epsilon;%Real load mass of system plant
