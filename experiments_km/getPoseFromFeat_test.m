function yaw = getPoseFromFeat_test(feat)
% GETPOSEFROMFEAT_TEST  Obtains an angle prediction, given a feature
% vector. Version that uses only raw features to predict yaw.


% Declaring global variables
globals;

indices = 43:63;

feat3 = feat(indices);
feat3 = 1./(1+exp(-feat3));
feat3 = bsxfun(@rdivide,feat3,sum(feat3));

N3 = length(indices);
% Initialize a vector whose dimensionality is same as that of feat3
locMax = false(size(feat3));
% For each dimension of feat3, verify if it is a local maximum, i.e., if it
% has a higher than the bins preceeding or succeeding it (wrapping around)
for j=1:N3
    locMax(j) = feat3(j) >= feat3(mod(j-2,N3)+1) & feat3(j) >= feat3(mod(j,N3)+1);
end
% If not a local maximum, set all those indices to -Inf
feat3(~locMax)=-Inf;

[~, ind] = max(feat3);

yaw = ind;
% yaw = (yaw - 11)*pi/10.5;


end