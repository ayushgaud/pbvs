function [] = mkdirOptional(dirName)
%MKDIROPTIONAL  Checks if directory with name DIRNAME exists. If it doesn't
%   exist, it is created.

if(~exist(dirName,'dir'))
    mkdir(dirName)
end

end

