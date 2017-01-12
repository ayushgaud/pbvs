function [cnn_model] = initNeuralNet(proto, weights, inputSize)

% Initializes a CNN model using the prototxt file and the caffemodel file
% names passed as arguments

% Declaring global variables
globals;

% Path to the prototxt file of the deployment version of the net
protoFile = fullfile(prototxtDir, proto, 'deploy.prototxt');
% Path to the weight (.caffemodel) file
weightsFile = fullfile(snapshotsDir, 'finalSnapshots', [weights '.caffemodel']);

% Create an RCNN model
cnn_model = rcnn_create_model(protoFile, weightsFile);
% Initialize the model with the weights
cnn_model = rcnn_load_model(cnn_model);
% Default image input size is 224 x 224 x 3 (H x W x C)
% inputSize = 224;
% Initialize the input size of the CNN
cnn_model.cnn.input_size = inputSize;
padRatio = 0.00;

% Mean image (to be subtracted from the test image, for normalization)
meanNums = [102.9801, 115.9465, 122.7717]; % magical numbers given by Ross
for i=1:3
    meanIm(:,:,i) = ones(inputSize)*meanNums(i);
end
cnn_model.cnn.image_mean = single(meanIm);
cnn_model.cnn.batch_size = 1;

end