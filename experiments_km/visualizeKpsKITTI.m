
%% Note: Run extractKpsKITTI.m before running this script


%% Group the detections by image, for easy visualization

% Current index (of image in sequence)
curImgIdx = 0;
% Current index (of detection in image)
curDetIdx = 0;

% Filename corresponding to the previous image being processed
prevFileName = '';

% Cell to hold all detections
kpDetections = {};
% Array to hold detections only for the current image
curDetections = {};

% For each dataStruct
for idx = 1:length(dataStructs)
    % If the current image filename is different from the one that we've
    % been processing until now
    if ~strcmp(dataStructs{idx}.fileName, prevFileName)
        
        % If this is not the first index (idx), then we've completed
        % collecting detections from one image. Add them to kpDetections.
        if idx ~= 1
            kpDetections{curImgIdx} = curDetections;
        end
        
        % Update prevFileName
        prevFileName = dataStructs{idx}.fileName;
        % Update the index of the current image
        curImgIdx = curImgIdx + 1;
        % Update the detection index in the current image
        curDetIdx = 1;
        % Array to hold detections in the current image
        curDetections = {};
    else
        % Update curDetIdx
        curDetIdx = curDetIdx + 1;
    end
    % Append the current dataStruct to the appropriate array
    curDetections{curDetIdx} = dataStructs{idx};
end


%% Predict keypoint locations and plot them

% Whether or not to write video output
writeVideoOutput = true;

% Initialize the GUI
fig = figure(1);
imgFile = fullfile(kittiImageDir, sprintf('%06d.png', imageList(1)));
img = imread(imgFile);

% Create a folder to store the output images
if writeVideoOutput
    mkdir(sprintf('kp_results/seq%02d_%03d_%03d', sequenceNum, startImageId, endImageId));
    set(fig, 'PaperPositionMode', 'auto');
end

% Sum of all indices we've seen thus far
idxSum = 0;

% For each image
for idx = 1:length(kpDetections)
    
    curKpDetection = kpDetections{idx};
    
    % Read in the current image
    im = imread(fullfile(curKpDetection{1}.fileName));
    
    % Draw the image
    imshow(im);
    hold on;
    % Plot title
    text(size(im,2)/2,3,sprintf('Keypoint Demo'),'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % Legend
    text(0,00,'Not occluded','color','g','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,30,'Partly occluded','color','y','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,60,'Fully occluded','color','r','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,90,'Unknown','color','w','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,120,'Don''t care region','color','c','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % Frame number
    text(size(im,2),0,sprintf('Sequence %d frame %d (%d/%d)', sequenceNum, imageList(idx), idx, length(imageList)),'color','g','HorizontalAlignment','right','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % Specifying colors for occlusion levels
    occlusionColors = {'g', 'y', 'r', 'w', 'c'};
    
    for j = 1:length(curKpDetection)
        
        % Increment idxSum
        idxSum = idxSum + 1;
        
        testFeat = featAll(idxSum, :);
        bbox = curKpDetection{j}.bbox;
        
        [kpCoords,scores] = maxLocationPredict(testFeat, bbox, params.heatMapDims);
        kpCoords = kpCoords(1:2, 1:14);
        [b, bi] = sort(scores, 'descend');
        [~, ind] = find(scores >= 0.4);
        kpCoords = kpCoords(1:2, ind);
        
        bbox2(1) = bbox(1); bbox2(2) = bbox(2); bbox2(3) = bbox(3)-bbox(1); bbox2(4) = bbox(4)-bbox(2);
        scatter(kpCoords(1,:),kpCoords(2,:),50,'r','filled')
        rectangle('Position', bbox2, 'LineWidth', 3, 'EdgeColor', 'g');
        
    end
    
    if writeVideoOutput
        print(sprintf('kp_results/seq%02d_%03d_%03d/%03d.png', sequenceNum, startImageId, endImageId, imageList(idx)), '-dpng', '-r0');
    end
    
    hold off;
    pause(0.1);
    
end


%% Old (primitive) GUI

% % For each detection in the image set
% for idx = 1:length(dataStructs)
%     
%     % Read in the image
%     im = imread(fullfile(dataStructs{idx}.fileName));
%     testFeat = featAll(idx,:);
%     bbox = dataStructs{idx}.bbox;
%     
%     [kpCoords,scores] = maxLocationPredict(testFeat, bbox, params.heatMapDims);
%     kpCoords = kpCoords(1:2, 1:14);
%     [b, bi] = sort(scores, 'descend');
%     [~, ind] = find(scores >= 0.8);
%     kpCoords = kpCoords(1:2, ind);
%     
%     bbox2(1) = bbox(1); bbox2(2) = bbox(2); bbox2(3) = bbox(3)-bbox(1); bbox2(4) = bbox(4)-bbox(2);
%     imshow(im);
%     hold on
%     scatter(kpCoords(1,:),kpCoords(2,:),50,'r','filled')
%     % scatter(kps(:,1),kps(:,2),50,'b','filled')
%     rectangle('Position', bbox2, 'LineWidth', 3, 'EdgeColor', 'g');
%     hold off
%     
%     pause;
% end