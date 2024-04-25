function ptCloud = helperProcessPointCloud(ptCloud)
%helperProcessPointCloud Process point cloud
%   helperProcessPointCloud processes a lidar point cloud by only keeping
%   points within some radius of the sensor and removing the ground plane.
%
%   See also pointCloud, findNeighborsInRadius, segmentGroundFromLidarData.

% Copyright 2021 The MathWorks, Inc.


% Discard points too close to the sensor
sensorOrigin = [0 0 0];
minRadius    = 3.5;       % in meters
ptCloud = select(ptCloud, ...
    setdiff(1:ptCloud.Count,findNeighborsInRadius(ptCloud, sensorOrigin, minRadius)), ...
    'OutputSize', 'full');

% Segment and remove ground
% Set the limits to select the point cloud
maxRadius    = 50;
selectLimitX = maxRadius*[-1 1];
selectLimitY = maxRadius*[-1 1];

% In order to not include ground in the processing, add a buffer value to
% the lower Z limit of the point cloud. Clip anything above 5m.
minZValue = ptCloud.ZLimits(1) + 0.2;
maxZValue = 5;
selectLimitZ = [minZValue maxZValue];

roi = [selectLimitX selectLimitY selectLimitZ];
indices = findPointsInROI(ptCloud,roi);
ptCloud = select(ptCloud,indices);

end