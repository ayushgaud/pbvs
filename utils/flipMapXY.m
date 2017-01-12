function [heatMap] = flipMapXY(heatMap,dims)
%FLIPMAP  Flips the X and Y components of each individual heatmap.
%   For example, the 'car' class has 14 keypoints. Say, we're talking about
%   6 x 6 heatmaps here. Then, we have 14 such 6 x 6 heatmaps concatenated
%   into a 504 element vector. We first reshape that vector into a 6 x 6 x
%   14 matrix, and then transpose the 6 x 6 component of each of the 14
%   channels.

% If the dimensions of the heatmap are not passed as an argument, assume
% defaults as specified in params.
if(nargin<2)
    globals;
    dims = params.heatMapDims;
end

% Compute the number of keypoints present, given the dimensions of each
% heatmap
nKps = size(heatMap,2)/(dims(1)*dims(2));
% For each of the feature vectors present in the batch
for i=1:size(heatMap,1)
    % Permute the heatmap, i.e., transpose (flip X and Y) of each heatmap
    % present in the current feature vector
    hMap = permute(reshape(heatMap(i,:),[dims(1),dims(2),nKps]),[2 1 3]);
    % Convert it back to a vector and store it in the same feature vector
    hMap = hMap(:);
    heatMap(i,:) = hMap';
end

end