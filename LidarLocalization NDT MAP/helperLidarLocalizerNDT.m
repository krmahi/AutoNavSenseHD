function [ndtMap,x,y,yaw,estimatedPose] = helperLidarLocalizerNDT(xyzPoints,ndtMap,initPose)
% helperLidarLocalizer Localize the point cloud using an NDT map
% helperLidarLocalizerNDT estimates the current pose of the lidar mounted on the 
% ego vehicle, thus localizes it in a known NDT map. It uses the point cloud data from the
% lidar mounted on the ego vehicle and an initial estimate of the pose.
%
% Copyright 2021 The MathWorks, Inc.

% Preprocess point cloud
ptCloudCurr = pointCloud(xyzPoints);
ptCloudCurr = helperProcessPointCloud(ptCloudCurr);

% Submap parameters
distThresh = 15;
submapSize = [30 30 5];

% Update submap if needed based on the initial pose estimate 
poseTranslation = initPose.Translation;
[isInside,distToEdge] = isInsideSubmap(ndtMap,poseTranslation);
submapNeedsUpdate = ~isInside ...       % Current pose is outside submap
    || any(distToEdge(1:2) < distThresh);   % Current pose is close to submap edge
if submapNeedsUpdate
    ndtMap = selectSubmap(ndtMap,poseTranslation,submapSize);
end

% Localize the point cloud in the map and find the pose
estimatedPose = findPose(ndtMap,ptCloudCurr,initPose,Tolerance=[0.1 0.05],OutlierRatio=0.55);
x = estimatedPose.Translation(1);
y = estimatedPose.Translation(2);
yaw = atan2(estimatedPose.Rotation(1,2),estimatedPose.Rotation(1,1));

end