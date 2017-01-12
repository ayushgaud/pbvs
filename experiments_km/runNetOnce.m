function [featVec] = runNetOnce(rcnn_model, dataStruct)
% RUNNETONCE  Runs a forward pass of the (deployment) net and returns the
% computed feature vector.
%   featureVec = RUNNETONCE(dataStruct, rcnn_model) returns in featureVec
%   the output feature vector.
%   INPUTS:
%       rcnn_model: the (initialized) CNN model
%       dataStruct: structure containing the file path, bounding box, and
%                   the class label
%   OUTPUT:
%       featureVec: the output of the CNN


% Declare global variables
globals;

% Make sure that caffe has been initialized for this model
if rcnn_model.cnn.init_key ~= caffe('get_init_key')
  error('You probably need to call rcnn_load_model');
end

%% Prepare data for running the net

% Read the input image
%img = imread(dataStruct.fileName);
img = dataStruct.image;
% Reorder the channels in the way Caffe expects (BGR)
img = single(img(:,:,[3 2 1]));

% Detection parameters
crop_mode = rcnn_model.detectors.crop_mode;
image_mean = rcnn_model.cnn.image_mean;
crop_size = size(image_mean,1);
crop_padding = rcnn_model.detectors.crop_padding;

% Crop the image to the desired input size
crop = rcnn_im_crop(img, dataStruct.bbox, crop_mode, crop_size, ...
    crop_padding, image_mean);

% Initialize a 4-D array to hold a single image
% ims = zeros(height, width, channels, number, datatype)
ims = zeros(crop_size, crop_size, 3, 1, 'single');

% Swap dims 1 and 2 to make width the fastest growing dimension
ims(:,:,:,1) = permute(crop, [2 1 3]);


%% Run the net

% Load the label
labels = zeros(1,1,1,1);
labels(1,1,1,1) = dataStruct.labels;

% Run a forward pass, using the label and image as inputs to the CNN
f = caffe('forward', {ims; single(labels)});
featVec = f{1};
featVec = featVec(:);


end