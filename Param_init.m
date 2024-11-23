%% Parameters and Vehicle model

M = 1200;     % Sprung mass kg
m1 = 100;     % Unsprung mass front kg
m2 = m1;      % Unsprung mass rear kg
I = 600;      % Inertia kgm2
Kf = 15000;   % Spring stiffness front N/m
Kr = Kf;      % spring stiffness rear N/m   
Ktf = 200000; % Tire spring stiffness front N/m
Ktr = Ktf;    % Tire spring stiffness rear N/m
Cf = 1200;    % Damping constant front Ns/m
Cr = Cf;      % Damping constant rear Ns/m
a = 1.2;      % CG distance from front m
b = 1.5;      % CG distance from rear m
g = 9.81;     % m/s^2
mu = 0.05;    % road deflection amplitude
v = 5.56;     % velocity of the vehicle
L = a+b;      % wheel base
Dist_param = [mu,L,v]; % Parameter vector

%% State Space Matrices

A = [0 1 0 0 0 0 0 0;
    0, (-(Cf+Cr))/M, 0, (a*Cf -b*Cr)/M,  -Kf/M, Cf/M, -Kr/M,  Cr/M;                          % heave dot
    0 0 0 1 0 0 0 0;
    0, (a*Cf - b*Cr)/I, 0, (-((a^2)*Cf) - ((b^2)*Cr))/I, a*Kf/I, -a*Cf/I, -b*Kr/I, b*Cf/I;   % pitch angle dot
    0 1 0 -a 0 -1 0 0;
    -Ktf/m1, Cf/m1, a*Ktf/m1, -a*Cf/m1, (Kf+Ktf)/m1, -Cf/m1, 0, 0;                           % z1 dot
    0 1 0 b 0 0 0 -1;
    -Ktr/m2, Cr/m2, -b*Ktr/m2, b*Cr/m2, 0, 0, (Kr+Ktr)/m2, -Cr/m2,;                          % z2 dot
    ];

B = [0        0       0       0;
    1/M     1/M       0       0;
     0        0       0       0;
   -a/I      b/I      0       0;
     0        0       0       0;
   -1/m1      0     Ktf/m1    0;
     0        0       0       0;
     0      -1/m2     0     Ktr/m2;];

C = [ 1 0 0 0 0 0 0 0;         % Z
      0 0 1 0 0 0 0 0;];       % Theta
D = 0;

Pss = ss(A,B, C, D);
P = tf(Pss);

Pss.StateName = {'Z', 'Z_dot', 'Theta', 'Theta_dot', 'Zsf - Z1', 'Z1_dot', 'Zsr - Z2', 'Z2_dot'};
Pss.InputName = {'Actuator Front', 'Actuator Rear', 'Disturbance Front', 'Disturbance Rear'};
Pss.OutputName = {'Z', 'Theta'};

save('Plant.mat','Pss','Dist_param')

isstable(Pss)
