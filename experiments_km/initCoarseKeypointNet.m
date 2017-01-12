%% Parameters for the Caffe model

% Name of the subfolder in the 'prototxts' directory where the deployment
% version of the network is defined, i.e., deploy.prototxt is present
proto_12kps = 'vggConv12Kps';
% Path to prototxt file of the deployment version of the net
protoFile_12kps = fullfile(prototxtDir, proto_12kps, 'deploySingle.prototxt');
% Name of the caffemodel file (excluding the '.caffemodel' suffix)
weightsFile_12kps = 'vggConv12Kps';
% Path to the snapshot of the net weights (caffemodel file)
binFile_12kps = fullfile(snapshotsDir, 'finalSnapshots', [weightsFile_12kps '.caffemodel']);

%% Initialize the model

% Create an RCNN model
cnn_model_conv12Kps = rcnn_create_model(protoFile_12kps, binFile_12kps);
% Initialize the model with the weights
cnn_model_conv12Kps = rcnn_load_model(cnn_model_conv12Kps);
% Default image input size is 224 x 224 x 3 (H x W x C)
inputSize_conv12Kps = 384;
% Initialize the input size of the CNN
cnn_model_conv12Kps.cnn.input_size = inputSize_conv12Kps;
padRatio = 0.00;

% Mean image (to be subtracted from the test image, for normalization)
meanNums = [102.9801, 115.9465, 122.7717]; % magical numbers given by Ross
meanIm = zeros(inputSize_conv12Kps, inputSize_conv12Kps, 3);
for i=1:3
    meanIm(:,:,i) = ones(inputSize_conv12Kps)*meanNums(i);
end
cnn_model_conv12Kps.cnn.image_mean = single(meanIm);
cnn_model_conv12Kps.cnn.batch_size = 1;
