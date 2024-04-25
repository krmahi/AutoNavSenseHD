function [img,cameraBoxes,cameraPose] = helperGetCameraData(dataLog,dataFolder)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

camIdx = 1; %Front Camera
cameraPose = dataLog.CameraData(camIdx).Pose;
cameraBoxes = dataLog.CameraData(camIdx).Detections;
imPath = fullfile(dataFolder,dataLog.CameraData(camIdx).ImagePath);
img = imread(imPath);
end