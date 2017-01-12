clear
clc
addpath(genpath('./'))

run ./rvctools/startup_rvc.m

more off; % turn off output paging

create_env=1;

global width;
global height;
global fx;
global fy;
global cx;
global cy;
fx = 396.17782;
fy = 396.17782;
cx = 322.453185;
cy = 174.243174;
width = 640;
height = 368;
scale = 3;
if(create_env)
    %setup
    try
    rosinit('http://127.0.0.1:11311');
    end
    img_sub = rossubscriber('/yolo/image','BufferSize', 1);
    %pos_sub = rossubscriber('/bebop/odom');
    pos_pub = rospublisher('/bebop/cmd_vel_1','geometry_msgs/Twist');
    msg = rosmessage(pos_pub);
    Tcurr=zeros(3,4);
end


imgd=imread('car.png');
imgd=imresize(imgd,[height width]);
try 
    cnn_model = evalin('base', 'cnn_model');
catch
    cnn_model = initViewpoint_NEW;
    assignin('base', 'cnn_model', cnn_model);
end
kpId=getfeature_PBVS_NEW(imgd, cnn_model);


%lambda   = 0.03;%stepsize - adjust for faster/slower convergence
lambda=2;
lambda2=0.2;
lambda_step=1.2;
iter = 0;
Z=3;
max_vel = 0.05;
limit = [max_vel, max_vel, max_vel];
while(1)
    tic
    iter=iter+1;
    fprintf('iter:%d\n',iter);
    
%     pos = receive(pos_sub);
%     eul = quat2eul([pos.Pose.Pose.Orientation.W pos.Pose.Pose.Orientation.X pos.Pose.Pose.Orientation.Y pos.Pose.Pose.Orientation.Z]);
%     Tcurr(1:3,1:3) = eul2rotm([eul(3),-eul(1),-eul(2)]); %ZYX->X -Z -Y
%     Tcurr(1:3,4) = [-pos.Pose.Pose.Position.Y, -pos.Pose.Pose.Position.Z, pos.Pose.Pose.Position.X];
    %pause(0.1);
    
    img=imcrop(readImage(receive(img_sub)),[0,0,width, height]);

    %imshow(img)
    if(max(max(max(img)))<=1)img=uint8(img*255);end

    kpI=getfeature_PBVS_NEW(img, cnn_model);

    %tulsiyani considers 0-360 interval;
    %we want -180-180 else the camera will take longer arc.
    
    if(kpI(1)>=0.8*2*pi) kpI(1)=kpI(1)-2*pi;
    elseif(kpI(1)>0.8*pi) kpI(1)=2*pi-kpI(1);
    end    
    if(kpId(1)>=0.8*2*pi) kpId(1)=kpId(1)-2*pi;
    elseif(kpId(1)>0.8*pi) kpId(1)=2*pi-kpId(1);
    end        
    
    
    err=-(kpI-kpId);%rotation of object=-pose of camera wrt object
    
    
    
    
    fprintf('yaw=%f\n',kpI(1)*180/pi);

    bbox=getboundingbox(img);

    center_x=double(bbox(3)+bbox(1))/2;
    center_y=double(bbox(4)+bbox(2))/2;
        
    xcurr=(center_x - cx)/fx;
    ycurr=(center_y - cy)/fy;
    
    theta=pi*kpI(1)/180
    dtheta=pi*err(1)/180
    %want to move in circle around object in x-z plane. 
    v(2)=-lambda2*ycurr*0;
    v(1)=-lambda*Z*cos(theta)*(dtheta) - lambda2*xcurr;
    v(3)=-lambda*Z*(-sin(theta))*(dtheta);
    v(4)=0;
    v(5)=-lambda*dtheta;
    v(6)=0;
    err_arr(iter) = err;
    vel_arr(iter,1:6) = v(1:6);
    theta=theta+v(5)
    %my convention to openrave conventions
%     v(1)=-v(1);
%     v(2)=-v(2);
%     v(3)=-v(3);
    
    %online depth estimation using center of bounding box.
%     if(iter>1)
%         Zinv=-((xcurr-xprev)+(1+xprev^2)*v(5))/v(1);
%         Z=1/Zinv;
%     end
    %Zarr(iter)=Z;
    %Z=mean(Z);
    Z = 3;
    %assignin('base','Zarr',Zarr);
    vprev=v;
    xprev=xcurr;
    
    subplot(2,2,1),imshow(img);
    title('current image');axis([0 width 0 height]);
    hold on
    scatter(center_x,center_y,20,'b','filled');
    scatter(cx,cy,20,'r','filled');
    hold off
    subplot(2,2,2),imagesc(single(rgb2gray(imgd))-single(rgb2gray(img)));title('image error');axis([0 width 0 height]);
    colormap('gray')
    title('image error');axis([0 width 0 height])
    
    normeError=norm(err);
    fprintf('|e|=%f\n',normeError);
    fprintf('v:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,Z=%.4f,|Tc|=%f\n',v(1),v(2),v(3),v(4),v(5),v(6),Z,sum(v.*v));
    
    %if(normeError*180/pi < 5 || iter > 400) break;end
    
    normv_arr(iter)=norm(v);
    subplot(2,2,3),plot(normv_arr);
    err_arr(iter)=normeError;title('velocity norm');
    
    subplot(2,2,4),plot(err_arr);title('Yaw error');
    %Tcurr(1:3,4)=Tcurr(1:3,4)+v(1:3);
    %Rot=rotz(v(6))*roty(v(5))*rotx(v(4));
    %Tdelta=[Rot [v(1);v(2);v(3)]; 0 0 0 1];
    %saveas(gcf,sprintf('images/%04d.png',iter));
    Tdelta = trnorm(delta2tr(v))    % differential motion
    Tnew=[Tcurr;0 0 0 1]*Tdelta;
%     Tnew = trnorm(Tnew);
%     %Tnew(3,4)=Tnew(3,4)-0.0050;
%     scale = 10;
%     %rot = rotm2eul(Tnew(1:3,1:3));
    des_pose = [Tdelta(1:3,4)]';
%     Tnew(1:3,4);
    des_pose = sign(des_pose).*min(abs(des_pose),limit)
    msg.Linear.X = -des_pose(3);
    msg.Linear.Y = des_pose(1);
    msg.Linear.Z = des_pose(2);
    msg.Angular.Z = -v(5);%-sign(v(5))*min(abs(v(5)),2*max_vel/Z);
    send(pos_pub,msg);
    %pause(0.1);
    
%     while(1)
%         pause(0.1);
%         pos = receive(pos_sub);
%         eul = quat2eul([pos.Pose.Pose.Orientation.W pos.Pose.Pose.Orientation.X pos.Pose.Pose.Orientation.Y pos.Pose.Pose.Orientation.Z]);
%         cur_pose = [-pos.Pose.Pose.Position.Y, -pos.Pose.Pose.Position.Z, pos.Pose.Pose.Position.X];
%         if abs(sum(cur_pose - des_pose)) < 3
%             break;
% %             if abs(sum(eul(1) + theta )) <0
% %             break;
% %             end
%         else
%            msg.Linear.X = des_pose(3);
%            msg.Linear.Y = -des_pose(2);
%            msg.Linear.Z = -des_pose(1);
%            msg.Angular.Z = -theta;
%            msg
%            send(pos_pub,msg);
%            pause(0.1);
%         end
%     end
      toc 
end