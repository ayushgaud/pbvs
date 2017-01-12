function [batches, batch_padding] = rcnnExtractRegionsSingleBox(dataStruct, rcnn_model,mirror,padRatio)

% Declare global variables
globals;

% If padding hasn't been specified, set it to zero
if(nargin<4)
    padRatio = 0;
end

% If the dataset isn't specified, assume it to be Pascal
imagesDir = pascalImagesDir;

% Set the directory containing images, depending on the dataset used
if strcmp(params.vpsDataset, 'imagenet')
    imagesDir = imagenetImagesDir;
elseif strcmp(params.vpsDataset, 'pascal')
    imagesDir = pascalImagesDir;
end

% If there's only one image, read it in
singleImage = length(dataStruct.voc_image_id) == 1;
if(singleImage)
    % Read the image in
    im = imread(fullfile(imagesDir,[dataStruct.voc_image_id{1} '.jpg']));
    
    im = single(im(:,:,[3 2 1]));
end

% Number of bboxes in the dataset
num_boxes = size(dataStruct.bbox,1);
% Batch size of the trained model
batch_size = rcnn_model.cnn.batch_size;
% Compute the number of batches required
num_batches = ceil(num_boxes / batch_size);
% Compute the number of images to be padded (in the last batch)
batch_padding = batch_size - mod(num_boxes-1, batch_size) - 1;

% Detection parameters
crop_mode = rcnn_model.detectors.crop_mode;
image_mean = rcnn_model.cnn.image_mean;
crop_size = size(image_mean,1);
crop_padding = rcnn_model.detectors.crop_padding;

% Create a cell to store batches
batches = cell(num_batches, 1);

% For each batch
for batch = 1:num_batches
    
    %  disp(batch);
    %parfor batch = 1:num_batches
    
    % Compute the start and end indixes for the batch
    batch_start = (batch-1)*batch_size+1;
    batch_end = min(num_boxes, batch_start+batch_size-1);
    
    % Create a 4-dimensional array to store all images of the batch
    % Eg. Typically crop_size = 224, batch_size = 20. So, we have a 224 x
    % 224 x 3 x 4 array
    ims = zeros(crop_size, crop_size, 3, batch_size, 'single');
    % For each batch
    for j = batch_start:batch_end
        % Print the filename of the image currently being processed
        % fullfile(imagesDir,[dataStruct.voc_image_id{j} '.jpg'])
        
        bbox = dataStruct.bbox(j,:);
        if(~singleImage)
            im = imread(fullfile(imagesDir,[dataStruct.voc_image_id{j} '.jpg']));
            % If the image has only one channel, replicate it (added by KM)
            if length(size(im)) == 2
                im = single(im(:,:,[1 1 1]));
            end
            % End of 'added by KM'
            im = single(im(:,:,[3 2 1]));
        end
        
        [crop] = rcnn_im_crop(im, bbox, crop_mode, crop_size, ...
            crop_padding, image_mean);
        if(mirror)
            %size(crop)
            for k=1:size(crop,3)
                crop(:,:,k) = fliplr(crop(:,:,k));
            end
        end
        % swap dims 1 and 2 to make width the fastest dimension (for caffe)
        ims(:,:,:,j-batch_start+1) = permute(crop, [2 1 3]);
    end
    
    batches{batch} = ims;
end
