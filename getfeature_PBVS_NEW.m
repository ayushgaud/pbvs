function [yaw, center_x, center_y]=getfeature_PBVS_NEW(img,cnn_model)
globals;
dataStruct.fileName=('');
dataStruct.bbox=single(getboundingbox(img));
dataStruct.labels=single(7);
dataStruct.image=img;



%initViewpointNet;
rcnn_model_vp=cnn_model;
featvp = runNetOnce(rcnn_model_vp, dataStruct);

% initKeypointNet;
% rcnn_model6=cnn_model_conv6Kps;
% feat = runNetOnce(rcnn_model6, dataStruct);
% feat = feat(1945:2448);
% feat = flipFeatVecXY(feat,[6 6]);
% feat6 = resizeHeatMapSingle(feat,[6 6]);
% 
% 
% initCoarseKeypointNet;
% rcnn_model12=cnn_model_conv12Kps;
% feat = runNetOnce(rcnn_model12, dataStruct);
% feat = feat(7777:9792);
% feat = flipFeatVecXY(feat,[12 12]);
% feat12 = resizeHeatMapSingle(feat,[12 12]);
% 
% posePriorFeat = computePosePriors(dataStruct, featvp);
% 
% 
% %feat=1./(1+exp(-feat12-feat6));
% feat = 1./(1+exp(-feat6-feat12-log(posePriorFeat+eps)));
% 
% %feat=1./(1+exp(-feat12));
% hDims = params.heatMapDims;
% predMethod = params.predMethod;
% params.alpha = 0.1;
% 
% [kpCoords,scores] = maxLocationPredict(feat,dataStruct.bbox,hDims);
% 
% s=kpCoords(:);
% kp=kpCoords;
% 
%scale=double(norm([dataStruct.bbox(3)-dataStruct.bbox(1) dataStruct.bbox(4)-dataStruct.bbox(2)]));
center_x=double(dataStruct.bbox(3)-dataStruct.bbox(1));
center_y=double(dataStruct.bbox(4)-dataStruct.bbox(2));

[rx, ry, rz]= mygetPoseFromFeat_test(featvp);
yaw=ry;

end