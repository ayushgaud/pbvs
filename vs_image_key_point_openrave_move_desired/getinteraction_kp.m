function Lsd=getinteraction_kp(s,cam,sd_len,Z)
%imgd=imgd';
KK=cam.K;
px = KK(1,1);
py = KK(2,2);
v0=KK(1,3)/2;
u0=KK(2,3)/2;
%img2=img(bord:size(imgd,1)-bord,bord:size(imgd,2)-bord);

%(Gx1 Gy1) = gradient(imgd);
%Gx = zeros(size(imgd));
%Gy = zeros(size(imgd));


% Gx=zeros(size(imgd));
% Gx(:,1:end-1) = (2047/8418)*(imgd(:,2:end) - imgd(:,1:end-1));
% Gx(:,1:end-2) = Gx(:,1:end-2)+(913/8418)*(imgd(:,3:end) - imgd(:,1:end-2));
% Gx(:,1:end-3) = Gx(:,1:end-3)+(913/8418)*(imgd(:,4:end) - imgd(:,1:end-3));
%
% Gy=double(zeros(size(imgd)));
% Gy(1:end-1,:) = (2047/8418)*(imgd(2:end,:) - imgd(1:end-1,:));
% Gy(1:end-2,:) = Gy(1:end-2,:)+(913/8418)*(imgd(3:end,:) - imgd(1:end-2,:));
% Gy(1:end-3,:) = Gy(1:end-3,:)+(913/8418)*(imgd(4:end,:) - imgd(1:end-3,:));



Lsd=zeros(sd_len,3);

for m=1:2:sd_len
    
    
    x = (s(m) - u0)/px ;
    y = (s(m+1) - v0)/py ;
    Zinv =  1/Z;
    
    Lsd(m,1) =  -Zinv;
    Lsd(m,2) =  0;
    Lsd(m,3) =  x*Zinv;
%     Lsd(m,4) =  x*y;
%     Lsd(m,5) =  -(1+x^2);
%     Lsd(m,6) =  y;

    
    
    Lsd(m+1,1) =  0;
    Lsd(m+1,2) =  -Zinv;
    Lsd(m+1,3) =  y*Zinv;
%     Lsd(m+1,4) = 1+y^2;
%     Lsd(m+1,5) = -x*y;
%     Lsd(m+1,6)  = -x;
    
    
end

end
