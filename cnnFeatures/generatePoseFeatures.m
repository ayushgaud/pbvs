function generatePoseFeatures(proto,suffix,inputSize,classInd,mirror)

% Declare global variables
globals;

% Dataset to extract features on (usually set to nothing, i.e., '')
dataSet = params.vpsDataset;

% Path to prototxt file of the deployment version of the net
protoFile = fullfile(prototxtDir,proto,'deploy.prototxt');
% Path to the snapshot of the net weights (caffemodel file)
binFile = fullfile(snapshotsDir,'finalSnapshots',[suffix '.caffemodel']);

% Create an RCNN model
cnn_model=rcnn_create_model(protoFile,binFile);
% Initialize the model with the weights
cnn_model=rcnn_load_model(cnn_model);
% Initialize the input size of the CNN
cnn_model.cnn.input_size = inputSize;
padRatio = 0.00;

% If mirror is set to 1, then set suffix to 'Mirror'
suff = '';
if(mirror)
    suff = 'Mirror';
end

% Mean Image (to be subtracted from the test image, for normalization)
meanNums = [102.9801,115.9465,122.7717]; %magical numbers given by Ross
for i=1:3
    meanIm(:,:,i) = ones(inputSize)*meanNums(i);
end
cnn_model.cnn.image_mean = single(meanIm);
cnn_model.cnn.batch_size=20;

%keyboard;

% Create directory to store the computed pose features
mkdirOptional(fullfile(cachedir,['rcnnPredsVps'],[proto suff]));
% For each class index chosen (about 12, by default)
for ind = classInd
    % Get the class label as a string
    class = pascalIndexClass(ind)
    % Get the rotation data for the Pascal dataset
    load(fullfile(rotationPascalDataDir,class));
    
    % Get the image IDs and 2D bounding boxes for instances of this class
    tmp.voc_image_id = {rotationData(:).voc_image_id};
    tmp.bbox = vertcat(rotationData(:).bbox);
    
    %tmp.bbox(:,3:4) = tmp.bbox(:,1:2)+tmp.bbox(:,3:4);
    %tmp.labels = ones(size(tmp.bbox,1),1)*ind;
    
    % Initialize labels for the bounding box to the index of this class
    tmp.labels = ones(size(tmp.bbox,1),1)*ind;
    
    %keyboard;
    
    % Create an RCNN bounding box for the ground truth bbox
    feat = rcnnFeaturesSingleBox(tmp,cnn_model,0,true);
    if(mirror)
        featMirror = rcnnFeaturesSingleBox(tmp,cnn_model,1,true);
        feat = addFeatMirrorFeat(feat,featMirror);
    end
    % keyboard;
    
    % Save the features for the class to the directory
    save(fullfile(cachedir,'rcnnPredsVps',[proto suff],class),'feat');
end

end

function feat = addFeatMirrorFeat(feat,featMirror)

permInds = [21:-1:1,22:42,63:-1:43,70:-1:64,71:77,84:-1:78];
feat(:,1:84) = (feat(:,1:84)+featMirror(:,permInds))/2;

end