function [ptCld,lidarBoxes,lidarPose] = helperGetLidarData(dataLog)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

ptCld = dataLog.LidarData.PointCloud;
lidarPose = dataLog.LidarData.Pose;
lidarBoxes = dataLog.LidarData.Detections;

end