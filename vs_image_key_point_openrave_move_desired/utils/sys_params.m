function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 0.18; % kg
g = 9.81; % m/s/s
% g =0.1; % m/s/s

I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = 0.086; % m

% Original
% params.minF = 0.0;
% params.maxF = 2.0*m*g;

params.minF = -2000.0*m*g;
params.maxF = 2000.0*m*g;


end
