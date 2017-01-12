try
rosinit('http://127.0.0.1:11311');
end
pos_pub = rospublisher('/bebop/cmd_vel','geometry_msgs/Twist');
pos_sub = rossubscriber('/bebop/cmd_vel_1');

iter = 0;
global pos;
pos = rosmessage(pos_pub);
while(1)
    try
        tic
        pos = receive(pos_sub,0.1);
        send(pos_pub,pos);
    catch
        if (toc < 2)
            send(pos_pub,pos);
        end
       
    end
    
    
end
