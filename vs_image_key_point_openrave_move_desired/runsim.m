close all;
clear;

addpath('utils');
%%%%%%%%%%%%%%%%%%%Update the changes done in the code here%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  waypoints changed
%  PD parameters changed
%  Terminate condition commented out
%
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pre-calculated trajectories
%   trajhandle = @traj_line;
%  trajhandle = @traj_helix;

% Trajectory generation with waypoints
% You need to implement this
%  trajhandle = @traj_generator;
 trajhandle = @calculate_desires;
% waypoints = [0    0   0;
%              1    2   1;
%              2    7   2;
%              3    -1  1;
%              4    8   -2;
%             -3  -2   1]';
      
% waypoints = [0    0   0;
%              1    0.5   0.75;
%              2    1   1.5;
%              3    0.75   1.0;
%              4    0.5   0.75;
%              5    0   0.5;
%              4    -0.5   0.75;
%              3    -0.75   1.0;
%              2    -1   1.5;
%              1    -0.5   0.75;]';
% waypoints=waypoint_generator('sine')

%UNCOMMENT BELOW FOR WAYPOINT RUNNING
% waypoints = [0    0   0;
%              1    0   0;
%              2    0   0;
%              3    0   0;
%              4    0   0;
%              5    0   0;
%              6    0   0;
%              7    0   0;
%              8    0   0;
%              9    0   0;]';
% %          
% trajhandle(0,zeros(13,1),waypoints);


%% controller
controlhandle = @controller_noPD;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);
