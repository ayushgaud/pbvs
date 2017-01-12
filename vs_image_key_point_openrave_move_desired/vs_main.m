clear
clc
addpath('/usr/share/openrave-0.8/matlab/')
addpath('/usr/share/openrave-0.8/matlab/examples')
addpath('openrave_mex')

%% Script for simulating a camera and displaying its results in real-time


more off; % turn off output paging
addopenravepaths()

if( ~exist('render','var') )
    render = [0];
end
sensorindex = 0;

%enter full path here
orEnvLoadScene('/home/gourav/Downloads/vs_image_key_point_openrave/openrave_data/couch.xml',1); % reset the scene
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
    fprintf('iter:%d\n',iter);
    iter=iter+1;
    data = orRobotSensorGetData(robotid, sensorindex);
    cam.T=data.T;
    cam.P=data.P;
    cam.K=data.KK;
    Tcurr=cam.T;
    T1=reshape(orBodyGetTransform(robots{1}.id), [3 4]);
    img=imresize(rgb2gray(data.I),[240,320]);
    
    
    
    % change the values for translation changes
    
    step=20;
    scale=1.2;
    T = maketform('affine', [1*scale 0 0; 0 1*scale 0; -scale*size(img,2)/2+size(img,2)/2 step-scale*size(img,1)/2+size(img,1)/2 1]);   %# represents translation
    if(einit<10)
    imgd = imtransform(img, T, ...
        'XData',[1 size(img,2)], 'YData',[1 size(img,1)],'FillValues',1);
    end
    
    subplot(2,2,1),imagesc(img);title('image');axis([0 320 0 240]);
    subplot(2,2,2),imagesc(imgd-img);title('image error');axis([0 320 0 240]);
    
    sI=getfeature_kp(img) ;
    sId=getfeature_kp(imgd) ;
    sd_len=length(sId);
    error=sI-sId;
    Ls=getinteraction_kp(sI,cam,sd_len,desired_depth);
    Lsd=getinteraction_kp(sId,cam,sd_len,desired_depth);
    if(norm(error)>10 && normv<0.1)mu=1;lambda=lambda*10;
    else mu=0;lambda=3;
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
    if(normeError < 1 || iter > 400) break;end
    
    normv_arr(iter)=norm(v);
    normv=norm(v);
    %subplot(2,2,3),plot(normv_arr);
    err_arr(iter)=norm(error);
    varr(iter,1:3)=v(1:3);
    subplot(2,2,3),plot(varr(:,1),'r');title('velocity norm');
    hold on
    subplot(2,2,3),plot(varr(:,2),'g');
    subplot(2,2,3),plot(varr(:,3),'b');
    hold off
    subplot(2,2,4),plot(err_arr);title('tot pixel error');
    %error'
    %v'
    v(3)=v(3)-0.0050;%required correction for openrave
    Tcurr(1:3,4)=Tcurr(1:3,4)+v(1:3);
    einit=100;
    %we are only considering translations
    %Rx = [1 0 0; 0 cos(v(4)) -sin(v(4)); 0 sin(v(4)) cos(v(4))];
    %Ry = [cos(v(5)) 0 sin(v(5)); 0 1 0; -sin(v(5)) 0 cos(v(5))];
    %Rz = [cos(v(6)) -sin(v(6)) 0; sin(v(6)) cos(v(6)) 0; 0 0 1];
    %Rot = Rx*Ry*Rz;
    %Td = trnorm(delta2tr(v));
    %Tcurr=Tcurr*Td;
    
    orBodySetTransform(robots{1}.id,Tcurr);
    pause(0.2)
end
