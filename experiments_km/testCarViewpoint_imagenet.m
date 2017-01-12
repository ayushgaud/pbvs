%% Test the viewpoint prediction for cars (on PASCALVOC)

% Define Classes
classes = {'aeroplane','bicycle','bird','boat','bottle','bus','car','cat','chair','cow','diningtable','dog','horse','motorbike','person','plant','sheep','sofa','train','tvmonitor'};
% Index for car class
classInds = 7;

% Test only on cars
numClasses = size(classInds,2);
% The pose prediction code can give multiple hypotheses. We want to evaluate only the top prediction here.
params.nHypotheses = 1;

% Create arrays to store errors and median errors
errors = zeros(numClasses,1);
medErrors = zeros(numClasses,1);

% Index for car class
c = 7;

% Get the name of the class as a string
class = classes{c};
disp(class);

% Regresses to pose and returns error
% [~,~,testErrs,testData] = regressToPose(class);
[testErrors, testMedErrors, testErrs, testData, testPreds, testLabels] = regressToPose(class);
% Get indices of samples that are not occluded or truncated
nonOccInds = ~(testData.occluded | testData.truncated);
% Compute errors on these indices
testErrsNonOcc = testErrs(nonOccInds);
% Get the median errors
medErr = median(testErrsNonOcc);
% Get errors for bin size theta (in degrees)
errBinSize = 30;
err = sum(testErrsNonOcc<=errBinSize)/numel(testErrsNonOcc);

% Display errors
err
medErr

% errors(c,:) = err;
% medErrors(c,:) = medErr;

% prettyPrintResults(errors,medErrors);