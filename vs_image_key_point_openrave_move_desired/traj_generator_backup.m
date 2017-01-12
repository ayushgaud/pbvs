function [ desired_state ] = traj_generator(t, state, waypoints)
%{ TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you must visited in order
% along the generated trajectory. 
% Assumption: Waypoints must be equi-distant. And approximate time with max
% velocity that we need to go from one point to next is known
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.

%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 1.5 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
%     coff=getCoff(waypoints);
%     scale=(t-traj_time(t_index))/(d0(t_index));
%     t0=polyT(8,0,scale)';
%     index=(t_index-1)*8+1:t_index*8;
%     desired_state.pos=coff(index,:)'*t0;
%     desired_state.vel=coff(index,:)'*t0*(1/d0(t_index));
%     desired_state.acc=coff(index,:)'*t0*(1/d0(t_index)^2);
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end
% function [T] = polyT(n,k,t)
%      
%         T = zeros(n,1);
%         D = zeros(n,1);
%         
%         % Initial
%         for i=1:n
%             D(i)=i-1;
%             T(i)=1;
%         end
%         
%         %Derivative
%         for  j= 1:k
%             for i= 1:n
%                 T(i)=T(i)*D(i);
%                 if D(i)>0
%                     D(i)=D(i)-1;
%                 end
%             end
%         end
%         
%         %put t value
%         for i=1:n
%             T(i)=T(i)*t^D(i);
%         end
%         T=T';
% end
% function[coff,A,b]=getCoff(waypoints)
%    n=size(waypoints,1)-1;
%    A=zeros(8*n,8*n);
%    b=zeros(1,8*n);
%    for i=1:n
%        b(:,i)=waypoints(:,i);
%        b(:,i+n)=waypoints(:,i+1);
%    end
%    
%    %constraint1 Pi(0)=Wi for all i=1...n
%    row=1;
%    for i=1:n
%        A(row,((i-1)*8)+1:i*8)=polyT(8,0,0);
%        row=row+1;
%    end
%     %constraint2 Pi(1)=Wi+1 for all i=1...n
%    for i=1:n
%        A(row,((i-1)*8)+1:i*8)=polyT(8,0,1);
%        row=row+1;
%    end
%    %constraint3 P1(k)(0)=0 for all 1<=k<=3
%    for i=1:n
%        for k=1:3
%            A(row,((i-1)*8)+1:i*8)=polyT(8,k,0);
%            row=row+1;
%        end
%    end
%     %constraint4 Pn(k)(1)=0 for all 1<=k<=3
%    for n=1:4
%        for k=1:3
%            A(row,((n-1)*8)+1:n*8)=polyT(8,k,1);
%            row=row+1;
%        end
%    end
%     %constraint5 Pi-1(k)(1)=Pi(k)(0) for all 1<=k<=3
%    for i=2:n
%        for k=1:6
%            A(row,((n-1)*16)+1:i*16)=[polyT(8,k,1)-polyT(8,k,0)];
%        end
%    end
%    coff=pinv(A)*b';
% end
% 
%    