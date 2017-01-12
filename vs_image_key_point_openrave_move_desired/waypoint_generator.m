function [ waypoints ] = waypoint_generator(trajectory)

if (strcmp(trajectory,'sine')==1)
    waypoints=zeros(3,100);
    for i=1:100
        x=i*0.75;
        y=0;
        z=1.5*sin(i);        
        waypoints(:,i) =[x,y,z];
    end
else
    waypoints = [0    0   0;
             1    0.5   0.75;
             2    1   1.5;
             3    0.75   1.0;
             4    0.5   0.75;
             5    0   0.5;
             4    -0.5   0.75;
             3    -0.75   1.0;
             2    -1   1.5;
             1    -0.5   0.75;]';
end
    
waypoints;
end
