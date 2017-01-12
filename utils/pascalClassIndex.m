function [i] = pascalClassIndex(class)
% PASCALCLASSINDEX  Given a class label, PASCALCLASSINDEX returns the index
% of the class according to the Pascal dataset.

classes = {'aeroplane','bicycle','bird','boat','bottle','bus','car','cat','chair','cow','diningtable','dog','horse','motorbike','person','pottedplant','sheep','sofa','train','tvmonitor'};
i = find(ismember(classes,{class}));

end