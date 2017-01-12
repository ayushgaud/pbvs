function [class] = pascalIndexToClassLabel(c)
% PASCALINDEXTOCLASSLABEL  Given a class index, PASCALINDEXTOCLASSLABEL 
% returns the label of the class according to the Pascal dataset.

classes = {'aeroplane','bicycle','bird','boat','bottle','bus','car','cat','chair','cow','diningtable','dog','horse','motorbike','person','pottedplant','sheep','sofa','train','tvmonitor','cup'};
if(c < 1 || c > length(classes))
    class = 'none';
else
    class = classes{c};
end

end
