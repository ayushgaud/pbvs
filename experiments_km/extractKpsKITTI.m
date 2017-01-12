%% Script to extract keypoints on KITTI (batch mode)

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


%% Parameters for KITTI (test data)

% ID of the sequence to be processed
sequenceNum = 5;

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
startImageId = 0;
% ID of the last image to process (in the sequence specified)
% endImageId = 100;
endImageId = numFrames-1;
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

% Create a cell to store data structs
dataStructs = {};

% Number of car detections thus far
numDetections = 0;

% Whether or not to save extracted features
saveKpFeats = true;


%% Initialize the data structs

% For each image in the list
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
        
        % Tracklet for the current frame (usually comprises of multiple
        % annotations, again referred to as tracklets)
        tracklet = tracklets{imageList(idx)+1};
        
        for j = 1:length(tracklet)
            % Current tracklet (annotation corresponding to a detection)
            curTracklet = tracklet(j);
            if ~strcmp(curTracklet.type, 'Car') && ~strcmp(curTracklet.type, 'Van') %|| ~ismember(curTracklet.id,carIds)
                continue
            end
            
            % Get the bounding box (x1,y1,x2,y2), required by the CNN
            bbox = single([curTracklet.x1, curTracklet.y1, curTracklet.x2, curTracklet.y2]);
            % Get the bounding box (x,y,w,h), required by the rect command
            bboxPos = int16([curTracklet.x1, curTracklet.y1, (curTracklet.x2-curTracklet.x1+1), (curTracklet.y2-curTracklet.y1+1)]);
            % Determine whether or not the object is occluded
            occluded = curTracklet.occlusion;
            % Determine whether or not the object is truncated
            truncated = curTracklet.truncation;
            
            if truncated ~= 0 || occluded ~= 0
                continue
            end
            
            numDetections = numDetections + 1;
            
            % Create the data structure for the current detection
            curDataStruct.bbox = bbox;
            curDataStruct.fileName = imgFile;
            curDataStruct.labels = single(pascalClassIndex(class));
            curDataStruct.carId = curTracklet.id;
            % Image number in the sequence
            curDataStruct.imgNo = idx;
            % Detection number in the image
            curDataStruct.detNo = j;
            
            dataStructs{numDetections} = curDataStruct;
            
        end
        
    end
    
end


%% Compute pose features for each detection

disp('Computing viewpoint features');

% Initialize the network used for viewpoint estimation
initViewpointNet;

poseFeats = [];
% For each detection in the image set
for idx = 1:length(dataStructs)
    % Current detection to be processed
    curDataStruct = dataStructs{idx};
    % Run the network on the detecion
    poseFeatTemp = runNetOnce(cnn_model, curDataStruct);
    if idx == 1
        poseFeats = zeros(length(dataStructs), length(poseFeatTemp));
    end
    poseFeats(idx,:) = poseFeatTemp';
end


%% Compute coarse and fine keypoint features for each detection

disp('Computing coarse keypoint features');

% Initialize the network used for coarse keypoint estimation
initCoarseKeypointNet;

% For each detection in the image set
for idx = 1:length(dataStructs)
    % Current detection to be processed
    curDataStruct = dataStructs{idx};
    % Run the network on the detection
    conv12FeatTemp = runNetOnce(cnn_model_conv12Kps, curDataStruct);
    % Extract only the part of the feature vector corresponding to the
    % 'car' class
    conv12FeatTemp = conv12FeatTemp(7777:9792);
    % In the first iteration, initialize a matrix to store the precomputed
    % features
    if idx == 1
        conv12Feats = zeros(length(dataStructs), length(conv12FeatTemp));
    end
    % Store the computed feature vector
    conv12Feats(idx, :) = conv12FeatTemp';
end


disp('Computing fine keypoint features');

% Initialize the network used for coarse keypoint estimation
initKeypointNet;

% For each detection in the image set
for idx = 1:length(dataStructs)
    % Current detection to be processed
    curDataStruct = dataStructs{idx};
    % Run the network on the detection
    conv6FeatTemp = runNetOnce(cnn_model_conv6Kps, curDataStruct);
    % Extract only the part of the feature vector corresponding to the
    % 'car' class
    conv6FeatTemp = conv6FeatTemp(1945:2448);
    % In the first iteration, initialize a matrix to store the precomputed
    % features
    if idx == 1
        conv6Feats = zeros(length(dataStructs), length(conv6FeatTemp));
    end
    % Store the computed feature vector
    conv6Feats(idx, :) = conv6FeatTemp';
end


%% Compute pose prior heatmaps for each detection

disp('Computing pose prior heatmaps');

% Number of keypoints for the 'car' class
numKps = 14;

% Initializing a matrix to store the pose prior heatmaps
posePriorFeats = zeros(length(dataStructs), params.heatMapDims(1)*params.heatMapDims(2)*numKps);

% Loading rotation data for Pascal VOC, if it does not already exist
if ~exist('rData', 'var')
    rData = load(fullfile(rotationPascalDataDir,class));
    rData = rData.rotationData;
end
% Extracting training samples and augmenting keypoints
if ~exist('trainData', 'var')
    trainData = rData(ismember({rData(:).voc_image_id},trainIds));
    trainData = augmentKps(trainData,dataStruct);
end

% For each detection in the image set
for idx = 1:length(dataStructs)
    % Current detection to be processed
    curDataStruct = dataStructs{idx};
    % Load the pose prediction for the current detection
    poseFeatTemp = poseFeats(idx,:);
    % Convert the pose prediction to a rotation matrix
    predTemp = predsToRotation(poseFeatTemp);
    predTemp = predTemp{1};
    % Compute the pose prior heatmaps
    pFeat = neighborMapsKpsScore(predTemp, curDataStruct.bbox, trainData);
    posePriorFeats(idx,:) = pFeat;
end


%% Concatenate all features

disp('Concatenating features');

% Upsample conv6 features to the specified size
conv6Feats = flipMapXY(conv6Feats, [6 6]);
conv6Feats = resizeHeatMap(conv6Feats, [6 6]);
% % Perform exponential normalization to normalize the feature maps
% conv6Feats = 1./(1+exp(-conv6Feats));

% Repeat the process for conv12 features
conv12Feats = flipMapXY(conv12Feats, [12 12]);
conv12Feats = resizeHeatMap(conv12Feats, [12 12]);
% conv12Feats = 1./(1+exp(-conv12Feats));

% Compose a feature map using all features (conv + pose)
featAll = 1./(1+exp(-conv6Feats-conv12Feats-log(posePriorFeats+eps)));

% Save keypoint features (featAll) if specified
if saveKpFeats
    save(fullfile(cachedir,'kpsDataKITTI', sprintf('seq%02d_%03d_%03d', sequenceNum, startImageId, endImageId)), 'featAll');
end
