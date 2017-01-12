%% Parameters for the Caffe model

% Name of the subfolder in the 'prototxts' directory where the deployment
% version of the network is defined, i.e., deploy.prototxt is present
proto = 'vggJointVps';
% Path to prototxt file of the deployment version of the net
protoFile = fullfile(prototxtDir, proto, 'deploySingle.prototxt');
% Name of the caffemodel file (excluding the '.caffemodel' suffix)
weightsFile = 'vggJointVps_iter_70000';
% Path to the snapshot of the net weights (caffemodel file)
binFile = fullfile(snapshotsDir, 'finalSnapshots', [weightsFile '.caffemodel']);

%% Initialize the model

% Create an RCNN model
cnn_model = rcnn_create_model(protoFile, binFile);
% Initialize the model with the weights
cnn_model = rcnn_load_model(cnn_model);
% Default image input size is 224 x 224 x 3 (H x W x C)
inputSize = 224;
% Initialize the input size of the CNN
cnn_model.cnn.input_size = inputSize;
padRatio = 0.00;

% Mean Image (to be subtracted from the test image, for normalization)
meanNums = [102.9801,115.9465,122.7717]; %magical numbers given by Ross
meanIm = zeros(inputSize, inputSize, 3);
for i=1:3
    meanIm(:,:,i) = ones(inputSize)*meanNums(i);
end
cnn_model.cnn.image_mean = single(meanIm);
cnn_model.cnn.batch_size=20;