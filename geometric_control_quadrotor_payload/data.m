%% data model
m_q= 2;  % quadrotor mass
Ixx=0.0211; %kg*m^2
Iyy=0.0211; %kg*m^2
Izz = 0.0366; %kg*m^2
J=  diag([Ixx, Iyy, Izz]);    %Inertia matrix of the quadrotor with respect to body frame
m_L=0.5;% Load mass nominal known from controller
m_L_plant = m_L; %Real load mass of system plant
L= 1;%free length of  spring
k=1e5; %stiffness cable known from controller
k_plant=k;%Real load mass of system plant
c=40; %damping cable known from controller
c_plant=c;%Real load mass of system plant