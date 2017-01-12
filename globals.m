%% Declaring global variables


%% Directories containing data

% Root directory of this codebase
global basedir
% Directory where data is cached. Typically, pretrained models, extracted
% features, pose predictions, etc. are available here.
global cachedir


% Pascal directories

% Root directory containing the Pascal3D dataset
global PASCAL3Ddir
% Base directory of Pascal VOC's development kit
global pascalDir
% Directory containing Pascal (VOC) images
global pascalImagesDir


% Imagenet directories

% Directory containing Imagenet images
global imagenetImagesDir


%% Directories containing annotations, computed features, etc.

% Directory containing viewpoint annotations generated over Pascal.
% Each annotations includes the image file name, bounding boxes detected,
% class label of each bounding box, and occlusion stats. Organized by image
% file name.
global rcnnVpsPascalDataDir
% Directory containing keypoint annotations generated for Pascal, organized
% by image file name.
global rcnnKpsPascalDataDir
% Directory containing viewpoint annotations for Imagenet, organized by
% image file name.
global rcnnVpsImagenetDataDir
% Directory containing viewpoint annotations for Imagenet, as well as
% Pascal, organized by image file name
global viewpointDataDir

% Directory containing rotation data for Pascal, organized by class
global rotationPascalDataDir
% Directory containing rotation data for Imagenet, organized by class
global rotationImagenetDataDir
% Directory containing joint rotation data (all three Euler angles),
% organized by class
global rotationJointDataDir

% Directory containing keypoint annotations for Pascal, organized by class
global kpsPascalDataDir

% Directory containing pascal image annotations, organized by image name
global annotationDir

% Contains keypoint annotations, as well as instance segmentation for
% Pascal images, organized by class
global segkpAnnotationDir

% Directory containing visualization
global websiteDir
% File containing RCNN detections on the validation set for Pascal
global rcnnDetectionsFile

% Directory containing results
global resultsDir


%% Directories containing CNN definitions, training metadata, etc.

% Directories specifying network architecture, trained models, and the like

% Directory containing snapshots of trained models
global snapshotsDir
% Directory containing prototxts of network models
global prototxtDir

% Directory containing training metadata for the keypoint and viewpoint
% networks

% Viewpoint training metadata
global finetuneVpsDir
global VNetTrainFilesDir
% Keypoint training metadata
global finetuneKpsDir


%% Parameters that determine a lot of other factors
% Eg. test data, keypoint/viewpoint detection methods to be used

% Contains a set of parameters to be passed to the function
global params
