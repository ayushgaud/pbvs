function [ desired_state ] = vel_exe( t, ~, comm)
    persistent scale t_pass t_last comm_obj send cur_state position;
    if(nargin == 1)
        desired_state.pos = zeros(3,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
        %velocity = zeros(6,1);
        position= zeros(4,1);
        comm_obj = comm; 
        send = 0;
        t_last = 0;
    else
        if isempty(comm_obj)
            comm_obj = tcpip('localhost', 30001, 'NetworkRole', 'client');
            fopen(comm_obj);
        end
        if comm_obj.BytesAvailable >= 16
            if comm_obj.BytesAvailable > 16
                fread(comm_obj, comm_obj.BytesAvailable - 16);
            end
            cur_state = evalin('base', 'state');
            position = fread(comm_obj, [4], 'float');
            max_vel = 0.03;
            scale = max([position(3); position(1); position(2)] - cur_state.pos)/max_vel;
            t_pass = t;
            t_last = t;
            send = 0;
        end
        if isempty(cur_state)
            cur_state.rot = zeros(3,1);
            cur_state.pos = zeros(3,1);
            cur_state.vel = zeros(3,1);
        end
        desired_state.pos = cur_state.pos;
        desired_state.vel = cur_state.vel;
        desired_state.acc = zeros(3,1);
        desired_state.yaw = cur_state.rot(3);
        desired_state.yawdot = 0;
        if (isempty(position))
            position= zeros(4,1);
        end
        
        try
        %cur_state = evalin('base', 'state');
        %(t-t_pass)/scale
        if ((t-t_pass)/scale) <= 1
        desired_state.pos = cur_state.pos + ((t-t_pass)/scale)*([position(3); position(1); position(2)] - cur_state.pos);
        desired_state.yaw = cur_state.rot(3) + (position(4)-cur_state.rot(3));
        send = send + 1;
        end
        catch
        end
        if t-t_last > 0.1
            cur_state = evalin('base', 'state');
             pos = [cur_state.pos; cur_state.rot(3)];
            fwrite(comm_obj,pos,'float');
            t_last = t; 
        end
    end
end