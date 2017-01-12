% Creates a video file from images in specified folder

% Load the directory path
% imgDir = fullfile('vp_results', 'seq01_000_446');
% imgDir = fullfile('vp_results', 'seq02_000_232');
% imgDir = fullfile('vp_results', 'seq03_000_143');
% imgDir = fullfile('vp_results', 'seq04_000_313');
% imgDir = fullfile('vp_results', 'seq05_000_296');
% imgDir = fullfile('vp_results', 'seq06_000_269');
% imgDir = fullfile('vp_results', 'seq07_000_799');
% imgDir = fullfile('vp_results', 'seq08_000_389');
% imgDir = fullfile('vp_results', 'seq10_000_293');
imgDir = fullfile('vp_results', 'seq18_000_338');
% Get directory listing
dirList = dir(imgDir);
% Remove the first two entries (as they correspond to '.' and '..')
dirList = dirList(3:end);

% Create a VideoWriter object
v = VideoWriter(fullfile([imgDir]));
% Set other parameters
v.FrameRate = 15;
v.Quality = 100;
% Open the video file to write frames
open(v);

% Go through each image in the directory and add it to the video
for i = 1:length(dirList)
    img = imread(fullfile(imgDir, dirList(i).name));
    writeVideo(v, img);
end

% Close the VideoWriter object
close(v);