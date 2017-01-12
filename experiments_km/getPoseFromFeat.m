function yaw = getPoseFromFeat(feat)
% GETPOSEFROMFEAT  Obtains an angle prediction, given a feature vector


% Declaring global variables
globals;

% Indices corresponding to feat1, feat2, feat3, feat1_coarse, feat2_coarse,
% feat3_coarse. Refer to prototxt files of vggJointVps for clarification.
indices{1} = 1:21;
indices{2} = 22:42;
indices{3} = 43:63;
indices{4} = 64:70;
indices{5} = 71:77;
indices{6} = 78:84;

% Scale of features (default is 1). Doesn't seem to affect predictions.
featScale = 1;

% Splitting feat into coarse and fine feature channels and normalizing them
% using the sigmoid function

% Fine features
feat1 = 1./(1+exp(-feat(indices{1})));
feat1 = bsxfun(@rdivide,feat1,sum(feat1));

feat2 = 1./(1+exp(-feat(indices{2})));
feat2 = bsxfun(@rdivide,feat2,sum(feat2));

feat3 = 1./(1+exp(-feat(indices{3})));
feat3 = bsxfun(@rdivide,feat3,sum(feat3));

% Coarse features
repInds = ceil(indices{1}/3);

feat1c = 1./(1+exp(-featScale*feat(indices{4})));
feat1c = feat1c(repInds);
feat1c = bsxfun(@rdivide,feat1c,sum(feat1c));

feat2c = 1./(1+exp(-featScale*feat(indices{5})));
feat2c = feat2c(repInds);
feat2c = bsxfun(@rdivide,feat2c,sum(feat2c));

feat3c = 1./(1+exp(-featScale*feat(indices{6})));
feat3c = feat3c(repInds);
feat3c = bsxfun(@rdivide,feat3c,sum(feat3c));

% Combine coarse and fine features, using alpha as the weight for the
% coarse features
alpha = 0;

feat1 = (1-alpha)*feat1 + alpha*feat1c;
feat2 = (1-alpha)*feat2 + alpha*feat2c;
feat3 = (1-alpha)*feat3 + alpha*feat3c;

% We're only looking at one hypothesis, for now
n = 1;

% Number of samples (only one, here)
N = 1;
% Dimensionality of feature
N3 = length(feat1);

% Get the indices of the maxima of feat1 and feat2, for each test feature
[~,I11] = max(feat1);
[~,I21] = max(feat2);

% Initialize a vector whose dimensionality is same as that of feat3
locMax = false(size(feat3));
% For each dimension of feat3, verify if it is a local maximum, i.e., if it
% has a higher than the bins preceeding or succeeding it (wrapping around)
for j=1:N3
    locMax(j) = feat3(j) >= feat3(mod(j-2,N3)+1) & feat3(j) >= feat3(mod(j,N3)+1);
end
% If not a local maximum, set all those indices to -Inf
feat3(~locMax)=-Inf;

% Create three arrays to indicate selected feature dimensions. Here, 'n'
% denotes the number of hypotheses we want to select per feature. The array
% stores the index of the dimension of feat3 which is selected.
selections1 = ones(N,n);
selections2 = ones(N,n);
selections3 = ones(N,n);

% Not used in the current edition of the code
simDiffs = normpdf([-10:10],0,1.5)/normpdf(0,0,1.5);

% For each sample, get feat3
f = feat3(:);

% Commented by Tulsiani
%for j=1:n
%    [~,k] = max(f);
%    selections3(i,j) = k;
%    diffk = 1-circshift(simDiffs,[0,k-11]);
%    f = f.*diffk;
%end
% End of 'Commented by Tulsiani'

% Sort the dimensions of feat3 (for the current sample) in descending
% order. Select the first n hypotheses (usually n = 1) corresponding to
% the highest local maximas of feat3.
[~,orders] = sort(f,'descend');
selections3(:) = orders(1:n);

% Index of the max component of feat1
I1s = [I11];
% Index of the max of feat2
I2s = [I21];
% Indices (currently in ascending order) ranging from 1 to N3
I3s = ones(N,1)*[1:N3];
    
% Get a linear indexing for I1s, I2s, and I3s
% I1 = I1s(sub2ind([N,size(I1s,2)],[1:N]',selections1(:,n)));
% I2 = I2s(sub2ind([N,size(I2s,2)],[1:N]',selections2(:,n)));
% I3 = I3s(sub2ind([N,size(I3s,2)],[1:N]',selections3(:,n)));

I1 = I1s;
I2 = I2s;
I3 = selections3;

% Obtain pose predictions
preds{n} = encodePose([(I1 - 11)*pi/10.5,(I2 - 11)*pi/10.5, (I3-0.5)*pi/10.5],params.angleEncoding);
pose = preds{1};

% Get the Euler angle representation of the prediction
eulerPred = decodePose(pose, params.angleEncoding);
% Get the equivalent rotation matrix representation
rotPred = encodePose(eulerPred, 'rot');
% Reshape the above 9 x 1 vector to a 3 x 3 matrix
rotPred = reshape(rotPred, 3, 3);

% Recover the angle
yaw = norm(logm(rotPred), 'fro');
yaw = yaw/sqrt(2)*180/pi;


end