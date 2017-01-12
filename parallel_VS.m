function [  ] = parallel_VS(  )
  %openrave convention
% +x  camera goes right
% +y  camera goes down
% +z  camera goes inside
% +w_x camera goes up
% +w_y camera goes right
% +w_z camera rotate clockwise

%control yaw only

%start openrave with > openrave --hidegui --serverport 17645


clear
clc

global orConnectionParams

%initialize openrave

addpath(genpath('./'))
%addpath('./openrave_mex')
%addpath('/home/gourav/Qualcom')
%addpath('./experiments_km')
%addpath(genpath('../ViewpointsAndKeypoints-km'))
run ./rvctools/startup_rvc.m
%run ../rvctools' (2)'/startup_rvc.m

more off; % turn off output paging
addopenravepaths()
global fx;
global fy;
global cx;
global cy;
fx = 640;
fy = 640;
cx = 320;
cy = 240;
if( ~exist('render','var') )
    render = [0];
end
sensorindex = 0;
orConnectionParams.ip='10.2.36.149';
orConnectionParams.port=17645

create_env=1;

if(create_env)
try   
    load('init_config.mat');
catch

%    orConnectionParams.port=4765
    %enter full path here
    orEnvLoadScene('/home/gourav/Qualcom/openrave_data/tridof.car_002.xml',1); % reset the scene
    orEnvLoadScene('/home/gourav/Qualcom/openrave_data/car/car003.xml',1); % reset the scene
    bodies = orEnvGetBodies();
    robots = orEnvGetRobots();
    robot=robots{1};
    robotid1 = robot.id
    
    %orRobotSetActiveManipulator(robot.id,robot.manip.name);
    % to turn on the rendering, send a command to the sensor
    for i = 0:1
        orRobotSensorConfigure(robot.id, i, 'PowerOn');
    end
    
    Tinit = orBodyGetTransform(robots{1}.id);
    Tinit =reshape(Tinit, [3 4]);
    Tinit(1,4)=Tinit(1,4)-70;
    
    Tinit(3,4)=Tinit(3,4)-1;
    robotid=robotid1;
    save('init_config.mat');
end
end
% comm_obj = tcpip('0.0.0.0', 30001, 'NetworkRole', 'server');    
% fopen(comm_obj);

disp('Communication established')

orBodySetTransform(robotid1,Tinit);
pause(1)
data = orRobotSensorGetData(robotid1, sensorindex);
Tcurr=Tinit;

imgd=imread('car2.png');
imgd=imresize(imgd,[480 640]);
try 
    cnn_model = evalin('base', 'cnn_model');
catch
    cnn_model = initViewpoint_NEW;
    assignin('base', 'cnn_model', cnn_model);
end
kpId=getfeature_PBVS_NEW(imgd, cnn_model);


%lambda   = 0.03;%stepsize - adjust for faster/slower convergence
lambda=5;
lambda_step=1.2;
iter   = 0;
Z=200;

while(1)
    
    iter=iter+1;
    fprintf('iter:%d\n',iter);
    data = orRobotSensorGetData(robotid, sensorindex);
    pause(0.1)
    Tcurr=data.T;
    
    img=data.I;
    if(max(max(max(img)))<=1)img=uint8(img*255);end
    kpI=getfeature_PBVS_NEW(img, cnn_model);
    
    %tulsiyani considers 0-360 interval;
    %we want -180-180 else the camera will take longer arc.
    
    if(kpI(1)>2*pi) kpI(1)=kpI(1)-2*pi;
    elseif(kpI(1)>pi) kpI(1)=2*pi-kpI(1);
    end    
    if(kpId(1)>2*pi) kpId(1)=kpId(1)-2*pi;
    elseif(kpId(1)>pi) kpId(1)=2*pi-kpId(1);
    end        
    
    
    err=-(kpI-kpId);%rotation of object=-pose of camera wrt object
    
    
    
    
    fprintf('yaw=%f\n',kpI(1)*180/pi);
    bbox=getboundingbox(img);
    center_x=double(bbox(3)+bbox(1))/2;
    center_y=double(bbox(4)+bbox(2))/2;
    
    xcurr=(center_x - cx)/fx
    ycurr=(center_y - cy)/fy
    
    theta=pi*kpI(1)/180
    dtheta=pi*err(1)/180
    %want to move in circle around object in x-z plane. 
    v(2)=-10*ycurr;
    v(1)=-lambda*Z*cos(theta)*(dtheta) - 10*xcurr;
    v(3)=-lambda*Z*(-sin(theta))*(dtheta);
    v(4)=0;
    v(5)=-lambda*dtheta;
    v(6)=0;
    
    theta=theta+v(5)
    %my convention to openrave conventions
    v(1)=-v(1);
    v(2)=-v(2);
    v(3)=-v(3);
    
    %online depth estimation using center of bounding box.
    if(iter>1)
        Zinv=-((xcurr-xprev)+(1+xprev^2)*v(5))/v(1);
        Z=1/Zinv;
    end
    Zarr(iter)=Z;
    %Z=mean(Zarr);
    Z = 300;
    assignin('base','Zarr',Zarr);
    vprev=v;
    xprev=xcurr;
    
    subplot(2,2,1),imshow(img);
    title('current image');axis([0 640 0 480]);
    hold on
    scatter(center_x,center_y,20,'b','filled');
    scatter(cx,cy,20,'r','filled');
    
    hold off
    subplot(2,2,2),imagesc(single(rgb2gray(imgd))-single(rgb2gray(img)));title('image error');axis([0 640 0 480]);
    colormap('gray')
    title('image error');axis([0 640 0 480])
    
    normeError=norm(err);
    fprintf('|e|=%f\n',normeError);
    fprintf('v:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,Z=%.4f,|Tc|=%f\n',v(1),v(2),v(3),v(4),v(5),v(6),Z,sum(v.*v));
    
    if(normeError*180/pi < 5 || iter > 400) break;end
    
    normv_arr(iter)=norm(v);
    subplot(2,2,3),plot(normv_arr);
    err_arr(iter)=normeError;title('velocity norm');
    
    subplot(2,2,4),plot(err_arr);title('tot pixel error');
    %Tcurr(1:3,4)=Tcurr(1:3,4)+v(1:3);
    %Rot=rotz(v(6))*roty(v(5))*rotx(v(4));
    %Tdelta=[Rot [v(1);v(2);v(3)]; 0 0 0 1];
    
    Tdelta = trnorm(delta2tr(v));    % differential motion
    Tnew=[Tcurr;0 0 0 1]*Tdelta;
    Tnew = trnorm(Tnew);
    Tnew(3,4)=Tnew(3,4)-0.0050;
    scale = 150;
    rot = rotm2eul(Tnew(1:3,1:3));
%    fwrite(comm_obj, [Tnew(1:3,4)/scale;rot(2)],'float');
%    Ttest=[Tcurr;0 0 0 1];
%     while(1)
%     if comm_obj.BytesAvailable >= 16
%         if comm_obj.BytesAvailable > 16
%             fread(comm_obj, comm_obj.BytesAvailable - 16);
%         end
%         pos = fread(comm_obj, [4], 'float');
%         %pos(4)
%         %acos(Ttest(1,1))
%         Ttest(1:3,4) = scale*[pos(2), pos(3), pos(1)];
%         Ttest(1:3,1:3) = [cos(pos(4)), 0, sin(pos(4)); 0, 1, 0; -sin(pos(4)), 0, cos(pos(4))];
%     end
%     if abs(sum(Ttest(1:3,4) - Tnew(1:3,4))) < 3
%         break;
%     else
%         %fwrite(comm_obj, [Tnew(1:3,4)/scale;theta],'float');
%         pause(0.1);
%         Ttest(1:3,4);
%         Tnew(1:3,4);
%         
%     end
%     end
    %cam.T=cam.T*Tdelta;
    orBodySetTransform(robots{1}.id,Tnew(1:3,:));
    pause(0.5)   
end

end