%1.changed terminate condition which will always be true
%2.




clear
clc
addpath('/usr/share/openrave-0.8/matlab/')
addpath('/usr/share/openrave-0.8/matlab/examples')
addpath('openrave_mex')

addpath('utils');
global flag;
flag = 0;

trajhandle = @calculate_desires;
controlhandle = @controller_noPD;
real_time = true;

disp('Initializing figures...');
h_fig = figure;
subplot(3,3,[1 2]);
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%transformation matrix from openrave to simulation

OR2SIM=[1 0 0;0 0 -1;0 1 0];
OR2SIM_inv= inv(OR2SIM);

%% *********************** INITIAL CONDITIONS ***********************
max_time=60;
params = sys_params;
disp('Setting initial conditions...');
tstep    = 0.01*1; % this determines the time step at which the solution is given
cstep    = 0.05*1; % image capture time interval
max_iteration = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
x0    = init_state(des_start.pos, 0);
xtraj = zeros(max_iteration*nstep, length(x0));
ttraj = zeros(max_iteration*nstep, 1);

x       = x0;        % state
iteration=1;
pos_tol = 0.01;
vel_tol = 0.01;

disp('Simulation Running....');
% Main loop
%for iteration = 1:max_iteration

%     timeint = time:tstep:time+cstep;

%   
% global flag;
% flag = iteration;
%      if iteration == 1
%         QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iteration, h_3d);
%         current_state = stateToQd(x);
%         fprintf('Time passed: %d',time);
%         desired_state = trajhandle(time, current_state);
%         QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
%         h_title = title(sprintf('iterationation: %d, time: %4.2f', iteration, time));
%     end
    % Run simulation for quad
    
    global accel;

%% Script for simulating a camera and displaying its results in real-time


more off; % turn off output paging
addopenravepaths()

if( ~exist('render','var') )
    render = [0];
end
sensorindex = 0;

%enter full path here
orEnvLoadScene('/home/gourav/ViewpointsAndKeyPoints-km/vs_image_key_point_openrave/openrave_data/couch.xml',1); % reset the scene
bodies = orEnvGetBodies();
robots = orEnvGetRobots();
robot=robots{1};
robotid = robot.id;

%orRobotSetActiveManipulator(robot.id,robot.manip.name);
% to turn on the rendering, send a command to the sensor
for i = 0:1
    orRobotSensorConfigure(robot.id, i, 'PowerOn');
end

Tinit = orBodyGetTransform(robots{1}.id);
Tinit =reshape(Tinit, [3 4]);
Tinit(3,4)=Tinit(3,4)-1;

orBodySetTransform(robots{1}.id,Tinit);
pause(1)
data = orRobotSensorGetData(robotid, sensorindex);

cam.T=data.T;
cam.P=data.P;
cam.K=data.KK;

mu =  0.000;
lambda   = 300;%stepsize - adjust for faster/slower convergence
iter   = 1;
desired_depth=0.01; % We do not know correct depth
einit=0;
normv=1;
while(1)
    timeint = time:tstep:time+cstep;
    tic;   

flag = iteration;
     if iteration == 1
        QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iteration, h_3d);
        current_state = stateToQd(x);
        fprintf('Time passed: %d',time);
        desired_state = trajhandle(time, current_state);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
        h_title = title(sprintf('iterationation: %d, time: %4.2f', iteration, time));
    end
    
    
    fprintf('iter:%d\n',iter);
    iter=iter+1;
    data = orRobotSensorGetData(robotid, sensorindex);
    cam.T=data.T;
    cam.P=data.P;
    cam.K=data.KK;
    Tcurr=cam.T;
    T1=reshape(orBodyGetTransform(robots{1}.id), [3 4]);
    img=imresize(rgb2gray(data.I),[240,320]);
    %________________ 
    
 
    
    
    % change the values for translation changes
    
    step=20;
    scale=1.2;
    T = maketform('affine', [1*scale 0 0; 0 1*scale 0; -scale*size(img,2)/2+size(img,2)/2 step-scale*size(img,1)/2+size(img,1)/2 1]);   %# represents translation
    if(einit<10)
    imgd = imtransform(img, T, ...
        'XData',[1 size(img,2)], 'YData',[1 size(img,1)],'FillValues',1);
    end
    
    subplot(3,3,3),imagesc(img);title('image');axis([0 320 0 240]);
    subplot(3,3,4),imagesc(imgd-img);title('image error');axis([0 320 0 240]);
    
    sI=getfeature_kp(img) ;
    sId=getfeature_kp(imgd) ;
    sd_len=length(sId);
    error=sI-sId;
%     display('ERROR');
    
     error
    %%%%%Plotting the error%%%%%%%%%%%%%%%

    varerr = orBodyGetTransform(robots{1}.id);
    varerr =reshape(varerr, [3 4]);
    varerr(3,4)=varerr(3,4)-1;

    varError(iter,1:3)=OR2SIM*varerr(:,4);
    subplot(3,3,7),plot(varError(:,1),'r');title('error norm');
    hold on
    subplot(3,3,7),plot(varError(:,2),'g');
    subplot(3,3,7),plot(varError(:,3),'b');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Ls=getinteraction_kp(sI,cam,sd_len,desired_depth);
    Lsd=getinteraction_kp(sId,cam,sd_len,desired_depth);
    if(norm(error)>10 && normv<0.1)mu=1;lambda=lambda*10;
    else
        mu=0;lambda=3;
    end
    Hsd = Lsd'*Lsd;
    diagHsd = eye(size(Hsd,1)).*Hsd;
    H = pinv((mu * diagHsd) + Hsd);
    e = H * Lsd' *error ;
    %v = - lambda*e;
    v = -lambda*pinv(Lsd) *error ;
    normeError=norm(error);
    fprintf('|e|=%f\n',normeError);
    
    
    
    %v(4:6) = 0*v(4:6);
    
    %fprintf('v:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,|Tc|=%f\n',v(1),v(2),v(3),v(4),v(5),v(6),sum(v.*v));
    fprintf('v:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,|Tc|=%f\n',v(1),v(2),v(3),sum(v.*v));
    if(normeError < 1 || iter > 400) break;
    end
    
%     normv_arr(iter)=norm(v);
    normv=norm(v);
    %subplot(2,2,3),plot(normv_arr);
    err_arr(iter)=norm(error);
    varr(iter,1:3)=OR2SIM*v(1:3);
    subplot(3,3,5),plot(varr(:,1),'r');title('velocity norm');
    hold on
    subplot(3,3,5),plot(varr(:,2),'g');
    subplot(3,3,5),plot(varr(:,3),'b');
    hold off
    subplot(3,3,6),plot(err_arr);title('tot pixel error');
    %error'
    %v'
    v(3)=v(3)-0.0050;%required correction for openrave
    %%%%%%%%%%%%%%%%%   simulation_3d code below
    
   v_sim= OR2SIM*v;% transforming openrave command velocity to simulation frame
%     v_sim=[1;1;1]; 
   accel = (v_sim(1:3)-current_state.vel(1:3))/cstep;
    initial_state=current_state;
    fprintf('Simulation_3d code starting');
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params,initial_state), timeint, x);
    x    = xsave(end, :)';
    % Save to traj
    xtraj((iteration-1)*nstep+1:iteration*nstep,:) = xsave(1:end-1,:);
    ttraj((iteration-1)*nstep+1:iteration*nstep) = tsave(1:end-1);

    % Update quad plot
    current_state = stateToQd(x);
    dist_error= sqrt(sum((desired_state.pos-current_state.pos).^2));
    display(dist_error);
%     if(dist_error<0.05)
    desired_state = trajhandle(time + cstep, current_state);
    QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
    set(h_title, 'String', sprintf('iterationation: %d, time: %4.2f', iteration, time + cstep))

    time = time + cstep; % Update simulation time
    
    t = toc;
    % Check termination criterationia
    if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
        break
    end
    


    if(~isempty(err))
        error(err);
    end

    %disp('finished.')

    t_out = ttraj;
    s_out = xtraj;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Tcurr(1:3,4)=Tcurr(1:3,4)+v(1:3);
    desired_state.pos = x(1:3);
    %Tcurr(1:3,4)=Tcurr(1:3,4) +  desired_state.pos(1:3);
    Tcurr(1:3,4)= OR2SIM_inv*desired_state.pos(1:3);
    einit=100;%norm(error);
    %we are only considering translations
    %Rx = [1 0 0; 0 cos(v(4)) -sin(v(4)); 0 sin(v(4)) cos(v(4))];
    %Ry = [cos(v(5)) 0 sin(v(5)); 0 1 0; -sin(v(5)) 0 cos(v(5))];
    %Rz = [cos(v(6)) -sin(v(6)) 0; sin(v(6)) cos(v(6)) 0; 0 0 1];
    %Rot = Rx*Ry*Rz;
    %Td = trnorm(delta2tr(v));
    %Tcurr=Tcurr*Td;
    
    orBodySetTransform(robots{1}.id,Tcurr);
    pause(0.2)
    str = input('Press "Enter" to continue:','s');
    display(length(str));
    if(length(str)~=0)
        break;
    elseif(error<10)
        break;
    end
    
    iteration = iteration+1;
    flag = iteration;
end
        % Truncate xtraj and ttraj
    xtraj = xtraj(1:iteration*nstep,:);
    ttraj = ttraj(1:iteration*nstep);

    % Truncate saved variables
    QP.TruncateHist();

    % Plot position
    h_pos = figure('Name', ['Quad position']);
    plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
    plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
    % Plot velocity
    h_vel = figure('Name', ['Quad velocity']);
    plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
    plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');
    
    hold off
