%% Parameters for the Caffe model

% Name of the subfolder in the 'prototxts' directory where the deployment
% version of the network is defined, i.e., deploy.prototxt is present
proto_kps = 'vggConv6Kps';
% Path to prototxt file of the deployment version of the net
protoFile_kps = fullfile(prototxtDir, proto_kps, 'deploySingle.prototxt');
% Name of the caffemodel file (excluding the '.caffemodel' suffix)
weightsFile_kps = 'vggConv6Kps';
% Path to the snapshot of the net weights (caffemodel file)
binFile_kps = fullfile(snapshotsDir, 'finalSnapshots', [weightsFile_kps '.caffemodel']);

%% Initialize the model

% Create an RCNN model
cnn_model_conv6Kps = rcnn_create_model(protoFile_kps, binFile_kps);
% Initialize the model with the weights
cnn_model_conv6Kps = rcnn_load_model(cnn_model_conv6Kps);
% Default image input size is 224 x 224 x 3 (H x W x C)
inputSize_conv6Kps = 192;
% Initialize the input size of the CNN
cnn_model_conv6Kps.cnn.input_size = inputSize_conv6Kps;
padRatio = 0.00;

% Mean image (to be subtracted from the test image, for normalization)
meanNums = [102.9801, 115.9465, 122.7717]; % magical numbers given by Ross
meanIm = zeros(inputSize_conv6Kps, inputSize_conv6Kps, 3);
for i=1:3
    meanIm(:,:,i) = ones(inputSize_conv6Kps)*meanNums(i);
end
cnn_model_conv6Kps.cnn.image_mean = single(meanIm);
cnn_model_conv6Kps.cnn.batch_size = 1;
