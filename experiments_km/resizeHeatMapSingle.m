function [map] = resizeHeatMapSingle(featVec,dims,dimsOut)
% RESIZEHEATMAPSINGLE  Resizes the heatmap of given dimensions 'dims' to a set of
%   specified dimensions 'dimsOut'.

% Declaring global variables
globals;

% Interpolation method used in resizing of the heatmap
method = params.interpolationMethod;

% If output dimensions are not passed as arguments, use defaults as
% specified in params
if(nargin < 3)
    dimsOut = params.heatMapDims;
end

% Compute the number of keypoints present in each feature vector, given the
% dimensionality of each heatmap
nKps = size(featVec,1)/(dims(1)*dims(2));

% Initialize the map with NaNs
map = nan(1,nKps*dimsOut(1)*dimsOut(2));
% Ratios of input and output dimensions
xRatio = dims(1)/dimsOut(1);
yRatio = dims(2)/dimsOut(2);
% Create a meshgrid and perform interpolation
[Xq,Yq] = meshgrid(1:dimsOut(1),1:dimsOut(2));
Xq = (Xq - 0.5)*xRatio + 0.5;
Yq = (Yq - 0.5)*yRatio + 0.5;


% For each keypoint present in the feature vector
for k = 1:nKps
    % If we're not using 'nearest neighbor' interpolation
    if(~strcmp(method,'nearest'))
        % Get the heatmap corresponding to the current keypoint
        mapKp = featVec((k-1)*dims(1)*dims(2)+1:(k)*dims(1)*dims(2));
        % Reshape it to a 6 x 6 matrix
        mapKp = reshape(mapKp,dims(2),dims(1));
        % Perform interpolation using the specified method
        mapOut = interp2(mapKp,Xq,Yq,method);
        mapOut = mapOut(:);
        % Put it into map
        map((k-1)*dimsOut(1)*dimsOut(2)+1:(k)*dimsOut(1)*dimsOut(2)) = mapOut;
    end
    % For each location in the output heatmap
    for x=1:dimsOut(1)
        for y = 1:dimsOut(2)
            % Get the input location it would correspond to
            xIn = ceil(x*xRatio);
            yIn = ceil(y*yRatio);
            % Get the index of the location in the input and output
            % feature maps
            index = (k-1)*dimsOut(1)*dimsOut(2) + (x-1)*dimsOut(2) + y;
            indexIn = (k-1)*dims(1)*dims(2) + (xIn-1)*dims(2) + yIn;
            % If it is nan, set it to the value at the input heatmap
            if(isnan(map(index)))
                map(index) = featVec(indexIn);
            end
        end
    end
end


end
