function [ output_args ] = parallel_sim( )
clear all
close all
addpath('utils');

%%% pre-calculated trajectories
%trajhandle = @traj_line;
%trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this

comm_obj = tcpip('localhost', 30001, 'NetworkRole', 'client');    
trajhandle = @vel_exe;
trajhandle([],[],comm_obj);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

[t, state] = simulation_3d(trajhandle, controlhandle);

end

