function [] = extractRigidKeypointFeatures(classInds, inputSize, proto, suffix, batchSize)
% EXTRACTRIGIDKEYPOINTFEATURES  Extracts keypoint features for the current
% class on Pascal VOC using the network passed as input

% Inputs:
%       classInds: indices of classes for which keypoint features have to
%       be computed
%       inputSize: dimensions of the image to be passed to the net
%       proto: name of the subfolder in the 'prototxts' directory where the
%       deployment version of the net 'deploy.prototxt' is present
%       suffix: name of the caffemodel file containing the trained network
%       parameters (weights and biases)
%       batchSize: size of each test batch


% Declaring global variables
globals;

% Path to prototxt file
% protoFile = fullfile(prototxtDir, proto, 'deploy.prototxt');
protoFile = fullfile(prototxtDir, proto, 'deploy.prototxt');
% Path to file containing the network weights
binFile = fullfile(snapshotsDir, 'finalSnapshots', [suffix '.caffemodel']);

% Create an RCNN model with the given network definition and snapshot file
cnn_model = rcnn_create_model(protoFile, binFile, 1);
cnn_model.cnn.batch_size = batchSize;
cnn_model = rcnn_load_model(cnn_model,1);
cnn_model.cnn.input_size = inputSize;

% Mean image provided by Ross Girshick (RCNN). Note that these values are
% the mean intensities per channel.
meanNums = [102.9801,115.9465,122.7717]; %magical numbers given by Ross
for i=1:3
    meanIm(:,:,i) = ones(inputSize)*meanNums(i);
end

% Create an image using the mean values
cnn_model.cnn.image_mean = single(meanIm);

% Initialize a data structure to hold a few details about the keypoints
key = keypointKey();
% Set the batch size for the images (this is the batch size we finally
% adopt for our network. But note that deploy.prototxt needn't be changed
% to accomodate this batch size. It is taken care of in the function
% rcnnFeaturesSingleBox).
imgBatchSize = 4*cnn_model.cnn.batch_size;

% Total number of keypoints predicted
N = key.totKps;

% For each class
for c = classInds
    % Get the label of the current class, given the object
    class = pascalIndexClass(c);
    % Load keypoint data for the current class (dataStruct)
    load(fullfile(kpsPascalDataDir,class));
    % Number of iterations to run
    numIters = ceil(length(dataStruct.voc_image_id)/imgBatchSize);
    % Matrix to hold computed features
    feat = [];
    
    % Get the index where keypoints of the current class start
    st = getfield(key.start,class);
    % Get the number of keypoints of the current class
    n = getfield(key.numKps,class);

    % Run iterations and compute features (used for keypoint estimation)
    for iter = 1:numIters
        % Start image
        start = (iter-1)*imgBatchSize+1;
        % End image
        last = min(iter*imgBatchSize,length(dataStruct.voc_image_id));
        % Initialize a data struct to pass to the net
        tmp.bbox = dataStruct.bbox(start:last,:);
        tmp.voc_image_id = dataStruct.voc_image_id(start:last);
        % Get feature vectors for the current batch of images using the CNN
        % model passed in as argument
        featTmp = rcnnFeaturesSingleBox(tmp, cnn_model);
        % Number of feature maps present
        mapSize = size(featTmp,2)/N;
        goodFeat = (st-1)*mapSize+[1:(n*mapSize)];
        % Concatenate it to the feat matrix
        feat = vertcat(feat,featTmp(:,goodFeat));
    end

    % Save the predictions to disk
    saveDir = fullfile(cachedir,'rcnnPredsKps',suffix);
    mkdirOptional(saveDir);
    save(fullfile(saveDir,class),'dataStruct','feat');
    fprintf('done\n');
end

end


% Function that returns a data struct containing information about
% keypoints
function key = keypointKey()

key = keypointKeyPascal();

end


% Function that returns a data struct containing information about
% keypoints of Pascal data
function key = keypointKeyPascal()

% Start indices of keypoints for various classes
start.aeroplane = 1;
start.bicycle = 17;
start.boat = 28;
start.bottle = 39;
start.bus = 47;
start.car = 55;
start.chair = 69;
start.diningtable = 79;
start.motorbike = 87;
start.sofa = 97;
start.train = 109;
start.tvmonitor = 116;

% Appending start indices to the struct
key.start = start;

% Number of keypoints for each class
start.aeroplane = 16;
start.bicycle = 11;
start.boat = 11;
start.bottle = 8;
start.bus = 8;
start.car = 14;
start.chair = 10;
start.diningtable = 8;
start.motorbike = 10;
start.sofa = 12;
start.train = 7;
start.tvmonitor = 8;

% Appending the number of keypoint of each class to the struct
key.numKps = start;
% Total number of keypoints
key.totKps = 123;

end
