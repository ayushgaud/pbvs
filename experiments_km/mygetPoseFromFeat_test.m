function [rx, ry, rz] = mygetPoseFromFeat_test(feat)
% GETPOSEFROMFEAT_TEST  Obtains an angle prediction, given a feature
% vector. Version that uses only raw features to predict yaw.
numBins = 21;
thetaInds = 1:21;
elevationInds = 22:42;
azimuthInds = 43:63;
predFeatAz = feat(azimuthInds);
predFeatEl = feat(elevationInds);
predFeatTh = feat(thetaInds);

[~,predAz] = max(predFeatAz);
[~,predEl] = max(predFeatEl);
[~,predTh] = max(predFeatTh);

pred = [(predTh-11) (predEl-11) (predAz-0.5)]*2*pi/numBins;
%encodePose([(I1 - 11)*pi/10.5,(I2 - 11)*pi/10.5, (I3-0.5)*pi/10.5],params.angleEncoding)
%rx=pred(1);
%ry=pred(2);
%rz=pred(3);
rx=pred(2);
ry=pred(3);
rz=pred(1);
%encodePose([rx ry rz],'rot')
%angle2dcm(rx,ry-pi/2,-rz,'ZXZ')
end