% Load conv6 features
disp('Loading conv6')
load(fullfile(cachedir,'rcnnPredsKps', [params.kpsNet 'Conv6Kps'],class));
feat = flipMapXY(feat,[6 6]);
feat6 = resizeHeatMap(feat,[6 6]);
featConv6 = 1./(1+exp(-feat6));

% Load conv12 features
disp('Loading conv12')
load(fullfile(cachedir,'rcnnPredsKps', [params.kpsNet 'Conv12Kps'],class));
feat = flipMapXY(feat,[12 12]);
feat12 = resizeHeatMap(feat,[12 12]);
featConv12 = 1./(1+exp(-feat12));

% Load pose priors
% [priorFeat] = posePrior(dataStruct,class,fnamesTrain);
priorFeat = posePrior(dataStruct, class, trainIds);

% Compose a feature map using all features (conv + pose)
featAll = 1./(1+exp(-feat6 - feat12 - log(priorFeat+eps)));

% Compose a feature map using pose features
featConv = 1./(1+exp(-feat6 - feat12));