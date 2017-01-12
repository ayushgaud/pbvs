function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thurst

% ORIGINAL VALUES
% kpx=32;
% kdx=2.4;
% kpy=32;
% kdy=3.2;
% kpz=800;
% kdz=60;
% kpphi=160;
% kdphi=2.6;
% kptheta=160;
% kdtheta=2.6;
% kppsi=160;
% kdpsi=2.6;
% 22 iterations
% %Set 1
% kpx=16*2.5;
% kpy=16*2.5;
% kpz=400*3;

kdx=2.4*8;
kdy=3.2*8;
kdz=60*4;

kix = 0.01;
kiy = 0.01;
kiz = 0.05;


kpphi=80;
kdphi=2.6;
kptheta=80;
kdtheta=2.6;
kppsi=80;
kdpsi=2.6;
%Set 2
kpx=0;
% kdx=0;
kpy=0;
% kdy=0;
kpz=0;
% kdz=0;
% kpphi=60;
% kdphi=2.6;
% kptheta=60;
% kdtheta=2.6;
% kppsi=60;
% kdpsi=2.6;

fprintf('current vel Vx=%d Vy=%d Vz=%d \n',state.vel(1),state.vel(2),state.vel(3));

persistent vel_error;

if isempty(vel_error)
   vel_error = [0,0,0]';
end

p=des_state.pos-state.pos;
v=des_state.vel-state.vel;
vel_error = v+vel_error;
% des_psi=des_state.yaw;
% des_psi_dot=des_state.yaw_dot;
r1c=des_state.acc(1)+kdx*v(1)+kpx*p(1)+ kix*vel_error(1);
r2c=des_state.acc(2)+kdy*v(2)+kpy*p(2)+ kiy*vel_error(2);
r3c=des_state.acc(3)+kdz*v(3)+kpz*p(3)+ kiz*vel_error(3);

des_phi=1/params.gravity*(r1c*des_state.yaw-r2c);
des_theta=1/params.gravity*(r1c+r2c*des_state.yaw);

F=params.mass*(params.gravity+r3c);


% Moment
M = zeros(3,1);

u_phi=kpphi*(des_phi-state.rot(1))+kdphi*(0-state.omega(1));

u_theta=kptheta*(des_theta-state.rot(2))+kdtheta*(0-state.omega(2));

u_psi=kppsi*(des_state.yaw-state.rot(3))+kdpsi*(0-state.omega(3));

M=[u_phi;u_theta;u_psi];

% =================== Your code ends here ===================

end
