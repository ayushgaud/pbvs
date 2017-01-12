function feat = rcnnFeaturesSingleBox(dataStruct, rcnn_model, mirror, labelsIn)

% dataStruct has voc_image_id, bbox, and label as fields
% rcnn_model is the cnn_model used to extract features

% Make sure that caffe has been initialized for this model
if rcnn_model.cnn.init_key ~= caffe('get_init_key')
  error('You probably need to call rcnn_load_model');
end
if(nargin<3)
    mirror=0;
end
if(nargin<4)
    labelsIn = false;
end

% Each batch contains 256 (default) image regions.
% Processing more than this many at once takes too much memory
% for a typical high-end GPU.
disp('Extracting image patches');
[batches, batch_padding] = rcnnExtractRegionsSingleBox(dataStruct, rcnn_model, mirror);
batch_size = rcnn_model.cnn.batch_size;
%disp(['numBatches = ' num2str(length(batches))])
%keyboard;

% Compute features for each batch of region images
feat_dim = -1;
% Initialize the list of features to an empty metrix
feat = [];
% Current image to be processed
curr = 1;
disp('Computing Features');

% For each batch
for j = 1:length(batches)
    % Forward propagate batch of region images
    
    % If labels are present in dataStruct, load them
    if(labelsIn)
        % Initialize labels for the current batch to zeros
        labels = zeros(1,1,1,batch_size);
        % Load labels for images in the current batch
        labels(1:min(length(dataStruct.labels)-(j-1)*batch_size,batch_size)) = dataStruct.labels(curr:min(j*batch_size,length(dataStruct.labels)));
        % Run a forward pass, using the label as input the the CNN
        f = caffe('forward', {batches{j};single(labels)});
        % If labels are not present, run a forward pass of the network
    else
        f = caffe('forward', batches(j));
    end
    
    f = f{1};
    f = f(:);
    
    % For the first batch, initialize feat_dim and feat
    if j == 1
        % Get the dimensionality of each feature
        feat_dim = length(f)/size(batches{j},4);
        % Create a matrix to store all features
        feat = zeros(size(dataStruct.bbox,1), feat_dim, 'single');
    end
    
    % Reshape f to get each image's features in a new row
    f = reshape(f, [feat_dim batch_size]);
    % For the last batch, trim f to size
    if j == length(batches)
        %disp(['padding = ' (num2str(batch_padding))])
        if batch_padding > 0
            f = f(:, 1:end-batch_padding);
        end
    end
    
    % Store computed features in f to feat
    feat(curr:curr+size(f,2)-1,:) = f';
    % Increment curr
    curr = curr + batch_size;
end
