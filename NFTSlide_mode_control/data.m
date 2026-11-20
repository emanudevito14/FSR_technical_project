%% data model
M=2; %quadrotor mass
m=0.5; %payload mass nominal
m_plant =m ; %real payload mass of plant
k=1e5;%cable stiffness nominal
k_plant=k; %real stiffness plant
c=40;%cable damping nominal
c_plant =c; %real damping plant
L_rest=1;%free length 
% Three-axis rotational inertia components of the quadrotor
Ixx=0.0211; 
Iyy=Ixx; 
Izz=0.0366;

J_rp=1.287e-4;%Rotor and propeller rotational inertia
cT=1.105e-5;% thrust coefficient
cM=1.779e-7;%drag moment coefficient
d=0.225;%arm length of the frame


