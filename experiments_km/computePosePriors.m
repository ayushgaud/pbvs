function posePriorFeat = computePosePriors(dataStruct, poseFeat)
% POSEPRIORFEAT  Computes the pose prior likelihood using training data,
% given the pose features (computed using a pose predictor, such as
% vggJointVps)

% Declaring global variables
globals;

% We're interested only in the 'car' class
class = 'car';
% Loading the train/test split for Pascal
load(fullfile(cachedir,'pascalTrainValIds'));
% Loading rotation data for Pascal VOC
rData = load(fullfile(rotationPascalDataDir,class));
rData = rData.rotationData;

% Augmenting information to dataStruct (hack to make it work)
dataStruct.kps{1} = 14;
dataStruct.voc_image_id = dataStruct.fileName;

% Extracting training samples
trainData = rData(ismember({rData(:).voc_image_id}, trainIds));
trainData = augmentKps(trainData,dataStruct);

% Get the height and width of the heatmap
H = params.heatMapDims(2);
W = params.heatMapDims(1);
% Number of keypoints for the current class
% Kp = size(trainData(1).kps,1);
Kp = 14;

% % Initialize a vector to hold prior likelihoods
% posePriorFeat = zeros(1,Kp*H*W);
% Compute prior features (requires poseFeat to be in the form of a rotation
% matrix)
poseFeat = predsToRotation(poseFeat');
poseFeat = poseFeat{1};
posePriorFeat = neighborMapsKpsScore(poseFeat, dataStruct.bbox, trainData);

end