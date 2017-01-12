function [accel]=openraveConnect(time , cstep, state)%desired_vels = {desired_vels.x  .y  .z  }//.phi  .theta .psi }

% open rave connection main file

%{

INPUT: time,cstep,state

OUTPUT: constant acceleration in each direction.

Comments:
Sending the position to open rave and getting the desired velocity for the 
next iteration

%}
global accel
 desired.velocity = [0.50,0.50,0.50]';

accel = (desired.velocity(1:3)-state.vel)./cstep;

accel;
end
