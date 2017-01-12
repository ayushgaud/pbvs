function [ terminate_cond ] = terminate_check( x, time, stop, pos_tol, vel_tol, time_tol)
%TERMINATE_CHECK Check termination criteria, including position, velocity and time

% Initialize
pos_check = true;
vel_check = true;
% pos_col_check = zeros(1, 3);
% 
% Check position and velocity and still time for each quad
% pos_check = pos_check && (norm(x(1:3) - stop) < pos_tol);
% vel_check = vel_check && (norm(x(4:6)) < vel_tol);
% pos_col_check(1,:) = x(1:3)';
% 
% str = input('Press "Enter" to continue:','s');
% display(length(str));
% Check total simulation time
% time_check = time > time_tol;
% 
% if (pos_check && vel_check)
%     terminate_cond = 1; % Robot reaches goal and stops, successful
% elseif time_check
%     terminate_cond = 2; % Robot doesn't reach goal within given time, not complete
% elseif (length(str)~=0)
%     terminate_cond = 3; % User wants to stop
% else
    terminate_cond = 0;
% end

end
