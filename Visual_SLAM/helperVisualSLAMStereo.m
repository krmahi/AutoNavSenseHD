function [mapPlot, optimizedPoses, addedFramesIdx] = helperVisualSLAMStereo(imdsLeft, imdsRight, intrinsics, maxDisparity, reprojectionMatrix)
%helperVisualSLAMStereo Evaluate the performance of a stereo visual SLAM algorithm
%   The implementation details of the stereo visual SLAM algorithm can be 
%   found in the Stereo Visual Simultaneous Localization and Mapping example.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2020-2022 The MathWorks, Inc.

% Set random seed for reproducibility
rng(0);

%% Map Initialization

% Read the first pair of stereo images
currFrameIdx   = 1;
currILeft      = readimage(imdsLeft, currFrameIdx);
currIRight     = readimage(imdsRight, currFrameIdx);

% Detect and extract ORB features from the rectified stereo images
scaleFactor = 1.2;
numLevels   = 8;
[currFeaturesLeft,  currPointsLeft]   = helperDetectAndExtractFeatures(currILeft, scaleFactor, numLevels); 
[currFeaturesRight, currPointsRight]  = helperDetectAndExtractFeatures(currIRight, scaleFactor, numLevels);

% Match feature points between the stereo images and get the 3-D world positions 
initialPose = rigidtform3d;
[xyzPoints, matchedPairs] = helperReconstructFromStereo(currILeft, currIRight, ...
    currFeaturesLeft, currFeaturesRight, currPointsLeft, currPointsRight, reprojectionMatrix, initialPose, maxDisparity);

%% Data Management and Visualization
% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;

% Add the first key frame
currKeyFrameId = 1;
vSetKeyFrames = addView(vSetKeyFrames, currKeyFrameId, initialPose, Points=currPointsLeft,...
    Features=currFeaturesLeft.Features);

% Add 3-D map points
[mapPointSet, stereoMapPointsIdx] = addWorldPoints(mapPointSet, xyzPoints);

% Add observations of the map points
mapPointSet = addCorrespondences(mapPointSet, currKeyFrameId, stereoMapPointsIdx, matchedPairs(:, 1));

% Update view direction and depth
mapPointSet = updateLimitsAndDirection(mapPointSet, stereoMapPointsIdx, vSetKeyFrames.Views);

% Update representative view
mapPointSet = updateRepresentativeView(mapPointSet, stereoMapPointsIdx, vSetKeyFrames.Views);

% Visualize matched features in the first key frame
featurePlot = helperVisualizeMatchedFeaturesStereo(currILeft, currIRight, currPointsLeft, ...
    currPointsRight, matchedPairs);

% Visualize initial map points and camera trajectory
mapPlot     = helperVisualizeSceneAndTrajectoryStereo(vSetKeyFrames, mapPointSet);

% Show legend
showLegend(mapPlot);

%% Initialize Place Recognition Database

% Load the bag of features data created offline
bofData         = load("bagOfFeaturesDataSLAM.mat");

% Initialize the place recognition database
loopDatabase    = invertedImageIndex(bofData.bof, SaveFeatureLocations=false);

% Add features of the first key frame to the database
addImageFeatures(loopDatabase, currFeaturesLeft, currKeyFrameId);

%% Tracking
% The tracking process is performed using every frame and determines when to 
% insert a new key frame. To simplify this example, we will terminate the tracking 
% process once a loop closure is found.

% ViewId of the last key frame
lastKeyFrameId    = currKeyFrameId;

% Index of the last key frame in the input image sequence
lastKeyFrameIdx   = currFrameIdx; 

% Indices of all the key frames in the input image sequence
addedFramesIdx    = lastKeyFrameIdx;

currFrameIdx      = 2;
isLoopClosed      = false;

% Main loop
isLastFrameKeyFrame = true;
while ~isLoopClosed && currFrameIdx <= numel(imdsLeft.Files)

    currILeft  = readimage(imdsLeft, currFrameIdx);
    currIRight = readimage(imdsRight, currFrameIdx);

    [currFeaturesLeft, currPointsLeft]    = helperDetectAndExtractFeatures(currILeft, scaleFactor, numLevels);
    [currFeaturesRight, currPointsRight]  = helperDetectAndExtractFeatures(currIRight, scaleFactor, numLevels);

    % Track the last key frame
    % trackedMapPointsIdx:  Indices of the map points observed in the current frame
    % trackedFeatureIdx:    Indices of the corresponding feature points in the current frame
    [currPose, trackedMapPointsIdx, trackedFeatureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
        vSetKeyFrames.Views, currFeaturesLeft, currPointsLeft, lastKeyFrameId, intrinsics, scaleFactor);
    
    if isempty(currPose) || numel(trackedMapPointsIdx) < 30
        currFrameIdx = currFrameIdx + 1;
        continue
    end
    
    % Track the local map and check if the current frame is a key frame.
    numSkipFrames     = 15;
    numPointsKeyFrame = 120;
    [localKeyFrameIds, currPose, trackedMapPointsIdx, trackedFeatureIdx, isKeyFrame] = ...
        helperTrackLocalMap(mapPointSet, vSetKeyFrames, trackedMapPointsIdx, ...
        trackedFeatureIdx, currPose, currFeaturesLeft, currPointsLeft, intrinsics, scaleFactor, numLevels, ...
        isLastFrameKeyFrame, lastKeyFrameIdx, currFrameIdx, numSkipFrames, numPointsKeyFrame);

    % Match feature points between the stereo images and get the 3-D world positions
    [xyzPoints, matchedPairs] = helperReconstructFromStereo(currILeft, currIRight, currFeaturesLeft, ...
        currFeaturesRight, currPointsLeft, currPointsRight, reprojectionMatrix, currPose, maxDisparity);  

    % Visualize matched features in the stereo image
    updatePlot(featurePlot, currILeft, currIRight, currPointsLeft, currPointsRight, trackedFeatureIdx, matchedPairs);

    if ~isKeyFrame
        currFrameIdx = currFrameIdx + 1;
        isLastFrameKeyFrame = false;
        continue
    else
        [untrackedFeatureIdx, ia] = setdiff(matchedPairs(:, 1), trackedFeatureIdx);
        xyzPoints = xyzPoints(ia, :);
        isLastFrameKeyFrame = true;
    end
    
    % Update current key frame ID
    currKeyFrameId  = currKeyFrameId + 1;

%% Local Mapping

    % Add the new key frame    
    [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
        currPose, currFeaturesLeft, currPointsLeft, trackedMapPointsIdx, trackedFeatureIdx, localKeyFrameIds);
        
    % Remove outlier map points that are observed in fewer than 3 key frames
    if currKeyFrameId == 2
        triangulatedMapPointsIdx = [];
    end
    
    mapPointSet = helperCullRecentMapPoints(mapPointSet, trackedMapPointsIdx, ...
        triangulatedMapPointsIdx, stereoMapPointsIdx);
    
    % Add new map points computed from disparity 
    [mapPointSet, stereoMapPointsIdx] = addWorldPoints(mapPointSet, xyzPoints);
    mapPointSet = addCorrespondences(mapPointSet, currKeyFrameId, stereoMapPointsIdx, ...
        untrackedFeatureIdx);
    
    % Create new map points by triangulation
    minNumMatches = 20;
    minParallax   = 0.35;
    [mapPointSet, vSetKeyFrames, triangulatedMapPointsIdx, stereoMapPointsIdx] = helperCreateNewMapPointsStereo( ...
        mapPointSet, vSetKeyFrames, currKeyFrameId, intrinsics, scaleFactor, minNumMatches, minParallax, ...
        untrackedFeatureIdx, stereoMapPointsIdx);

    % Local bundle adjustment
    [refinedViews, dist] = connectedViews(vSetKeyFrames, currKeyFrameId, MaxDistance=2);
    refinedKeyFrameIds = refinedViews.ViewId;

    % Always fix the first two key frames
    fixedViewIds = refinedKeyFrameIds(dist==2);
    fixedViewIds = fixedViewIds(1:min(10, numel(fixedViewIds)));

    % Refine local key frames and map points
    [mapPointSet, vSetKeyFrames, mapPointIdx] = bundleAdjustment(...
        mapPointSet, vSetKeyFrames, [refinedKeyFrameIds; currKeyFrameId], intrinsics, ...
        FixedViewIDs=fixedViewIds, PointsUndistorted=true, AbsoluteTolerance=1e-7,...
        RelativeTolerance=1e-16, Solver='preconditioned-conjugate-gradient', MaxIteration=10);

    % Update view direction and depth
    mapPointSet = updateLimitsAndDirection(mapPointSet, mapPointIdx, ...
        vSetKeyFrames.Views);

    % Update representative view
    mapPointSet = updateRepresentativeView(mapPointSet, mapPointIdx, ...
        vSetKeyFrames.Views);

    % Visualize 3-D world points and camera trajectory
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);

    %% Loop Closure
    % Check loop closure

    % Minimum number of feature matches of loop edges
    loopEdgeNumMatches = 60;

    % Detect possible loop closure key frame candidates
    [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
        loopDatabase, currILeft, loopEdgeNumMatches);

    if isDetected
        % Add loop closure connections
        isStereo = true;
        [isLoopClosed, mapPointSet, vSetKeyFrames, loopClosureEdge] = ...
            helperAddLoopConnections(...
            mapPointSet, vSetKeyFrames, validLoopCandidates, currKeyFrameId, ...
            currFeaturesLeft, currPointsLeft, loopEdgeNumMatches, isStereo);
    end

    % If no loop closure is detected, add current features into the database
    if ~isLoopClosed
        addImageFeatures(loopDatabase,  currFeaturesLeft, currKeyFrameId);
    end
    
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    addedFramesIdx  = [addedFramesIdx; currFrameIdx]; 
    currFrameIdx    = currFrameIdx + 1;
end % End of main loop

if ~isLoopClosed
    disp('Loop closure cannot be found');
    optimizedPoses = [];
    return
end

% Create a pose graph from the key frames set
G = createPoseGraph(vSetKeyFrames);

% Remove weak edges and keep loop closure edges
EG = rmedge(G, find(G.Edges.Weight < minNumMatches & ...
    ~ismember(G.Edges.EndNodes, loopClosureEdge, 'rows')));

% Optimize the pose graph
optimG = optimizePoseGraph(EG, 'g2o-levenberg-marquardt');
optimizedPoses = optimG.Nodes;

% Update the view poses
vSetKeyFramesOptim = updateView(vSetKeyFrames, optimizedPoses);

% Update map points after optimizing the poses
mapPointSet = helperUpdateGlobalMap(mapPointSet, vSetKeyFrames, vSetKeyFramesOptim);

updatePlot(mapPlot, vSetKeyFrames, mapPointSet);

% Plot the optimized camera trajectory
plotOptimizedTrajectory(mapPlot, optimizedPoses)

% Update legend
showLegend(mapPlot);
end

%--------------------------------------------------------------------------
function [features, validPoints] = helperDetectAndExtractFeatures(Irgb, ...
    scaleFactor, numLevels, varargin)
%helperDetectAndExtractFeatures detect and extract features

numPoints   = 1500;

% In this example, the images are already undistorted. In a general
% workflow, uncomment the following code to undistort the images.
%
% if nargin > 3
%     intrinsics = varargin{1};
% end
% Irgb  = undistortImage(Irgb, intrinsics);

% Detect ORB features
Igray  = rgb2gray(Irgb);

points = detectORBFeatures(Igray, ScaleFactor=scaleFactor, NumLevels=numLevels);

% Select a subset of features, uniformly distributed throughout the image
points = selectUniform(points, numPoints, size(Igray, 1:2));

% Extract features
[features, validPoints] = extractFeatures(Igray, points);
end

%--------------------------------------------------------------------------
function [xyzPoints, indexPairs] = helperReconstructFromStereo(I1, I2, ...
    features1, features2, points1, points2, reprojectionMatrix, currPose, maxDisparity)
%helperReconstructFromStereo reconstruct scene from stereo image using the disparity map

indexPairs     = helperFindValidFeaturePairs(features1, features2, points1, points2, maxDisparity);
disparityMap   = disparitySGM(rgb2gray(I1), rgb2gray(I2), DisparityRange=[0, maxDisparity], ...
    UniquenessThreshold=2);
xyzPointsAll   = reconstructScene(disparityMap, reprojectionMatrix);

% Find the corresponding world point of the matched feature points 
locations      = floor(points1.Location(indexPairs(:, 1), [2 1]));
xyzPoints      = [];
isPointFound   = false(size(points1));

for i = 1:size(locations, 1)
    point3d = squeeze(xyzPointsAll(locations(i,1), locations(i, 2), :))';
    isPointValid   = all(~isnan(point3d)) && all(isfinite(point3d)) &&  point3d(3) > 0;
    isDepthInRange = point3d(3) < 100/reprojectionMatrix(4, 3);
    if isPointValid && isDepthInRange
        xyzPoints       = [xyzPoints; point3d]; %#ok<*AGROW> 
        isPointFound(i) = true;
    end
end
indexPairs = indexPairs(isPointFound, :);
xyzPoints  = xyzPoints * currPose.Rotation + currPose.Translation;
end

%--------------------------------------------------------------------------
function indexPairs = helperFindValidFeaturePairs(features1, features2, points1, points2, maxDisparity)
%helperFindValidFeaturePairs match features between a pair of stereo images

indexPairs  = matchFeatures(features1, features2, Unique=true, MaxRatio=1, MatchThreshold=40);

matchedPoints1 = points1.Location(indexPairs(:,1), :);
matchedPoints2 = points2.Location(indexPairs(:,2), :);
scales1        = points1.Scale(indexPairs(:,1), :);
scales2        = points2.Scale(indexPairs(:,2), :);

dist2EpipolarLine = abs(matchedPoints2(:, 2) - matchedPoints1(:, 2));
shiftDist = matchedPoints1(:, 1) - matchedPoints2(:, 1);

isCloseToEpipolarline = dist2EpipolarLine < 2*scales2;
isDisparityValid      = shiftDist > 0 & shiftDist < maxDisparity;
isScaleIdentical      = scales1 == scales2;
indexPairs = indexPairs(isCloseToEpipolarline & isDisparityValid & isScaleIdentical, :);
end

%--------------------------------------------------------------------------
function mapPointSet = ...
    helperCullRecentMapPoints(mapPointSet, mapPointsIdx, newPointIdx, stereoMapPointsIndices)
%helperCullRecentMapPoints cull recently added map points
outlierIdx = setdiff([newPointIdx; stereoMapPointsIndices], mapPointsIdx);

if ~isempty(outlierIdx)
    mapPointSet   = removeWorldPoints(mapPointSet, outlierIdx);
end
end

%--------------------------------------------------------------------------
function mapPointSet = helperUpdateGlobalMap(mapPointSet, vSetKeyFrames, vSetKeyFramesOptim)
%helperUpdateGlobalMap update map points after pose graph optimization
posesOld     = vSetKeyFrames.Views.AbsolutePose;
posesNew     = vSetKeyFramesOptim.Views.AbsolutePose;
positionsOld = mapPointSet.WorldPoints;
positionsNew = positionsOld;
indices = 1:mapPointSet.Count;

% Update world location of each map point based on the new absolute pose of 
% the corresponding major view
for i = 1: mapPointSet.Count
    majorViewIds = mapPointSet.RepresentativeViewId(i);
    tform = rigidtform3d(posesNew(majorViewIds).A/posesOld(majorViewIds).A);
    positionsNew(i, :) = transformPointsForward(tform, positionsOld(i, :));
end
mapPointSet = updateWorldPoints(mapPointSet, indices, positionsNew);
end