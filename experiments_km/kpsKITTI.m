
% We're intereseted only in the 'car' class
class = 'car';

% Turn off Matlab warnings
warning('off', 'all');

% Declare global variables
globals;

% % Initialize the network for viewpoint prediction
% initViewpointNet;

% Add KITTI's Matlab code directory to path (the visualization one)
addpath /home/km/ViewpointsAndKeypoints/data/KITTI/devkit_tracking/matlab/

% Whether or not to write output to a video file
writeVideoOutput = true;

% Whether to show coarse (left, right, front, rear) or fine (angle in
% degrees) labels over the selected bounding boxes
showCoarseLabels = true;


%% Initialize the Neural Nets once

% cnn_model = initNeuralNet('vggJointVps', 'vggJointVps_iter_70000', 224);

% cnn_model_kps = initNeuralNet('vggConv6Kps', 'vggConv6Kps', 192);


%% Parameters for KITTI (test data)

% ID of the sequence to be processed
sequenceNum = 3;

% Mode ('manual', or 'auto'). Specifies if the user will input the bounding
% box or if they have to be picked up from the ground truth.
bboxMode = 'auto';


% Base directory (containing KITTI data)
kittiBaseDir = fullfile(basedir, 'data', 'KITTI');
% Root directory containing KITTI images (for training sequences)
kittiImageDir = fullfile(kittiBaseDir, sprintf('image_02/%04d', sequenceNum));
% Directory containing KITTI labels (for training sequences)
kittiLabelDir = fullfile(kittiBaseDir, 'label_02');
% Directory containing camera calibration data
kittiCalibDir = fullfile(kittiBaseDir, 'calib');

% Get number of images in the sequence
numFrames = length(dir(fullfile(kittiImageDir)))-2;

% ID of the first image to process (in the sequence specified)
startImageId = 1;
% ID of the last image to process (in the sequence specified)
endImageId = 2;
% endImageId = numFrames-1;
% Creating a list of images to process
imageList = startImageId:endImageId;
% ID(s) of the car to track
carIds = [9];

% % Create an array to store the predictions
% yawPreds = zeros(size(imageList));

% Get calibration parameters
% parameters: calib directory, sequence num, camera id (here, 2)
P = readCalibration(kittiCalibDir, sequenceNum, 2);

% Load labels for the current sequence
tracklets = readLabels(kittiLabelDir, sequenceNum);

% Initialize the GUI
fig = figure(1);
imgFile = fullfile(kittiImageDir, sprintf('%06d.png', imageList(1)));
img = imread(imgFile);

% Create a folder to store the output images
if writeVideoOutput
    mkdir(sprintf('kp_results/seq%02d_%03d_%03d', sequenceNum, startImageId, endImageId));
    set(fig, 'PaperPositionMode', 'auto');
end

% Predict pose for each image
for idx = 1:length(imageList)
    
    % Generate the file path for the current image to be processed
    imgFile = fullfile(kittiImageDir, sprintf('%06d.png',imageList(idx)));
    % Load the image
    img = imread(imgFile);
    
    if strcmp(bboxMode, 'manual')
        % Display the image, and wait for the user to draw a bounding box
        % around the object of interest
        imshow(img);
        r = imrect;
        position = wait(r);
        bbox = single([position(1), position(2), position(1)+position(3), position(2)+position(4)]);
    else
        
        % Draw the image
        imshow(img);
        hold on;
        % Plot title
        % text(size(img,2)/2,3,sprintf('Viewpoint Demo'),'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        text(size(img,2)/2,3,sprintf('Keypoint Demo'),'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        % Legend
        text(0,00,'Not occluded','color','g','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        text(0,30,'Partly occluded','color','y','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        text(0,60,'Fully occluded','color','r','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        text(0,90,'Unknown','color','w','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        text(0,120,'Don''t care region','color','c','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        
        % Frame number
        text(size(img,2),0,sprintf('Sequence %d frame %d (%d/%d)', sequenceNum, imageList(idx), idx, length(imageList)),'color','g','HorizontalAlignment','right','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
        
        % Specifying colors for occlusion levels
        occlusionColors = {'g', 'y', 'r', 'w', 'c'};
        
        % Tracklet for the current frame (usually comprises of multiple
        % annotations, again referred to as tracklets)
        tracklet = tracklets{imageList(idx)+1};
        
        for j = 1:length(tracklet)
            % Current tracklet (annotation corresponding to a detection)
            curTracklet = tracklet(j);
            if ~strcmp(curTracklet.type, 'Car') && ~strcmp(curTracklet.type, 'Van') %|| ~ismember(curTracklet.id,carIds)
                continue
            end
            
            % Get the bounding box (x1,y1,x2,y2), required by the CNN)
            bbox = single([curTracklet.x1, curTracklet.y1, curTracklet.x2, curTracklet.y2]);
            % Get the bounding box (x,y,w,h), required by the rect command
            bboxPos = int16([curTracklet.x1, curTracklet.y1, (curTracklet.x2-curTracklet.x1+1), (curTracklet.y2-curTracklet.y1+1)]);
            % Determine whether or not the object is occluded
            occluded = curTracklet.occlusion;
            % Determine whether or not the object is truncated
            truncated = curTracklet.truncation;
            % Draw the rectangle
            rectangle('Position', bboxPos, 'EdgeColor', occlusionColors{occluded+1}, 'LineWidth', 3);
            
            % If the current object is occluded or truncated, don't process
            if occluded > 0 || truncated > 0
                continue
            end
            
            % Create the data structure for the current detection
            dataStruct.bbox = bbox;
            dataStruct.fileName = imgFile;
            dataStruct.labels = single(pascalClassIndex(class));
            
            % Run the network on the detection (tracklet)
            initViewpointNet;
            featVec = runNetOnce(cnn_model, dataStruct);
            
            % Get pose from the feature vector
            % yaw = getPoseFromFeat(featVec);
            yaw = getPoseFromFeat_test(featVec);
            % Get ground truth yaw
            yaw_true = curTracklet.ry*180/pi;
            
            disp('Loading conv6');
            initKeypointNet;
            featVec_6Kps = runNetOnce(cnn_model_conv6Kps, dataStruct);
            featVec_temp = featVec_6Kps(1945:2448);
            feat = flipFeatVecXY(featVec_temp, [6,6]);
            feat6 = resizeHeatMapSingle(feat, [6 6], params.heatMapDims);
            featConv6 = 1./(1+exp(-feat6));
            
            disp('Loading conv12');
            initCoarseKeypointNet;
            featVec_12Kps = runNetOnce(cnn_model_conv12Kps, dataStruct);
            featVec_temp = featVec_12Kps(7777:9792);
            feat = flipFeatVecXY(featVec_temp, [12 12]);
            feat12 = resizeHeatMapSingle(featVec_temp, [12 12], params.heatMapDims);
            featConv12 = 1./(1+exp(-feat12));
            
            disp('Computing pose prior features');
            posePriorFeat = computePosePriors(dataStruct, featVec);
            % featPose = resizeHeatMapSingle(posePriorFeat, [12, 12], params.heatMapDims);
            
            % testFeat = featConv6 + featConv12;
            testFeat = 1./(1+exp(-feat6-feat12-posePriorFeat));
            
            disp('Predicting keypoints');
            [kpCoords,scores] = maxLocationPredict(featConv6, bbox, params.heatMapDims);
            scatter(kpCoords(1,:),kpCoords(2,:),50,'r','filled');
            
%             % Run the keypoint network on the detection (tracklet)
%             featVec_kps = runNetOnce(cnn_model_conv6Kps, dataStruct);
%             featVec_temp = featVec_kps(1945:2448);
%             % disp('Loading conv6');
%             feat = flipFeatVecXY(featVec_temp, [6,6]);
%             feat6 = resizeHeatMapSingle(feat, [6 6], params.heatMapDims);
%             featConv6 = 1./(1+exp(-feat6));
%             [kpCoords,scores] = maxLocationPredict(featConv6, bbox, params.heatMapDims);
%             kpCoords = kpCoords(1:2,1:14);
%             scatter(kpCoords(1,:),kpCoords(2,:),50,'r','filled');
            
            % Draw the label above the object
            
            if showCoarseLabels
                % Coarse label text
                if (yaw >= 18 && yaw <= 21) || (yaw >= 1 && yaw <= 3)
                    label_text = 'front';
                elseif (yaw >=4 && yaw <= 9)
                    label_text = 'left';
                elseif (yaw >= 10 && yaw <= 12)
                    label_text = 'rear';
                elseif(yaw >= 13 && yaw <= 17)
                    label_text = 'right';
                else
                    label_text = 'other';
                end
                
            else
                % Fine label test
                label_text = sprintf('(%02d) %3.2f %3.2f', curTracklet.id, yaw, yaw_true);
            end
            
            % label_text = sprintf('%3.2f', yaw);
            x = (curTracklet.x1 + curTracklet.x2)/2;
            y = curTracklet.y1;
            text(x, max(y-5,40), label_text, 'color', occlusionColors{occluded+1}, 'BackgroundColor', 'k', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'FontSize', 8);
            
        end
        
        if writeVideoOutput
            % Somehow doesn't work
            % saveas(fig, sprintf('vp_results/seq%02d_%03d_%03d/%03d.jpg', sequenceNum, startImageId, endImageId, imageList(i)));
            
            % Works
            % print(sprintf('vp_results/seq%02d_%03d_%03d/%03d.png', sequenceNum, startImageId, endImageId, imageList(i)), '-dpng', '-r0');
            print(sprintf('kp_results/seq%02d_%03d_%03d/%03d.png', sequenceNum, startImageId, endImageId, imageList(idx)), '-dpng', '-r0');
        end
        
        hold off;
        pause(0.1);
    end
    
end
    

