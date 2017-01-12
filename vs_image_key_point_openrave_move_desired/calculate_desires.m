function [desired_state]=calculate_desires(time,initial_state)%initial state has 

%{

INPUTS: time, constant acceleration, state

OUTPUTS:  desired position, velocity, acceleration

Assumption: We are limiting it to constant acceleration, and at the
junction there wil be impulsive change in the acceleration.
cstep to be taken as 0.05


For a small unit path we need the value of constant acceleration.
We need desired velocity and acceleration at each step
Time input will be absolute
Calculation of desired states at junctions.
%}

% Approach 1 using the constant acceleration for the unit cstep iteration.
 global accel;  
 global flag;
 display(flag);
 cstep = 0.05;
 normT = time;
%  normT = time - (flag-1).*cstep;
 
 desired_state.pos = zeros(3,1);
 desired_state.vel = zeros(3,1);
 desired_state.acc = zeros(3,1);
 desired_state.yaw = 0;
 
if(time==0)
    initial_state.vel=[0,0,0];
elseif(time == inf)
    initial_state.vel=[0,0,0];
else 
    normT = time - (flag-1)*cstep;
 desired_state.acc = accel; % acc is constant
 desired_state.vel = initial_state.vel+(desired_state.acc*normT); 
 desired_state.pos = initial_state.vel.*normT + (desired_state.acc)*((normT^2)*(0.5))+initial_state.pos;%ut+1/2*at^2
end
% Approach 2 using acceleration as dv/dt at calculating for each time

fprintf('time %d \n',normT);
% fprintf('initial vel');
fprintf('accel=%d \n',accel);
fprintf('initial vel Vx=%d Vy=%d Vz=%d \n',initial_state.vel(1),initial_state.vel(2),initial_state.vel(3));
fprintf('desired vel Vx=%d Vy=%d Vz=%d \n',desired_state.vel(1),desired_state.vel(2),desired_state.vel(3));
fprintf('desired acc Ax=%d Ay=%d Az=%d \n',desired_state.acc(1),desired_state.acc(2),desired_state.acc(3));
% fprintf('desired_state.pos');
% display (desired_state.pos);
% fprintf('desired_state.vel');
% display (desired_state.vel);
% fprintf('desired_state.acc');
% display (desired_state.acc);

desired_state;
end
