function bb=getboundingbox(img)
img = rgb2gray(img);
xmin=size(img,1);
ymin=size(img,2);
xmax=0;
ymax=0;
thr = 0.8*max(max(img));
%assuming 

for i=1:size(img,1)
    for j=1:size(img,2)
        
        if(img(i,j)>=thr)continue;end
        if(xmin>i)xmin=i;continue;end
        if(ymin>j)ymin=j;continue;end
        if(xmax<i)xmax=i;continue;end
        if(ymax<j)ymax=j;continue;end
            
    end
end

%bb=[xmin ymin xmax ymax];    
bb=[ymin xmin ymax xmax];    
% imshow(img)
% hold on
% rectangle('Position',[bb(1) bb(2) bb(3)-bb(1) bb(4)-bb(2)])
end

