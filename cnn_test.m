try
    rosinit('http://10.2.36.181:11311');
end
    img_sub = rossubscriber('/yolo/image','BufferSize', 5);
    pos_pub = rospublisher('/bebop2/command/trajectory','trajectory_msgs/MultiDOFJointTrajectory');
    msg = rosmessage(pos_pub);
    msg.Points = rosmessage('trajectory_msgs/MultiDOFJointTrajectoryPoint');
    msg.Points.Velocities = rosmessage('geometry_msgs/Twist');
    msg.Points.Accelerations = rosmessage('geometry_msgs/Twist');
    msg.Points.Transforms = rosmessage('geometry_msgs/Transform');
z =5;
angles = zeros(1,3);
theta_dot = 0.02;
try
    cnn_model = evalin('base', 'cnn_model');
catch
    cnn_model = initViewpoint_NEW;
    assignin('base', 'cnn_model', cnn_model);
end

while(1)
    angles(1) = angles(1) +  theta_dot + pi;
    angles(1) = mod(angles(1),pi);
    quat = eul2quat(angles);
   msg.Points.Transforms.Translation.Z =1;
    msg.Points.Transforms.Translation.X = (z/2) + z*cos(angles(1) - pi);
    msg.Points.Transforms.Translation.Y = z*sin(angles(1) - pi);
    msg.Points.Transforms.Rotation.W = quat(1);
    msg.Points.Transforms.Rotation.X = quat(2);
    msg.Points.Transforms.Rotation.Y = quat(3);
    msg.Points.Transforms.Rotation.Z = quat(4);
    
    img=imcrop(readImage(receive(img_sub)),[0,0,640, 368]);
    if(max(max(max(img)))<=1)img=uint8(img*255);end
    kpI=getfeature_PBVS_NEW(img, cnn_model)*180/pi
    send(pos_pub,msg);
    pause(0.1);
end
