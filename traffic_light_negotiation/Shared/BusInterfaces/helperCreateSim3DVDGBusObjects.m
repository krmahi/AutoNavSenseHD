% helperCreateSim3DVDGBusObjects creates bus objects used by the Sim3D VDG
% and Pack Lanes truth blocks in HighwayLaneFollowingRRTestBench.slx

% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2022-2023 The MathWorks, Inc.

function helperCreateSim3DVDGBusObjects(nvp)

    arguments
        % Total number of points from parent vehicle at which Sim 3D VDG calculates the lanes boundaries.  
        nvp.laneDimensions = 49;
    end

    % BusLaneBoundaries
    BusLaneBoundaries = Simulink.Bus;
    BusLaneBoundaries.Description = '';
    BusLaneBoundaries.DataScope = 'Auto';
    BusLaneBoundaries.HeaderFile = '';
    BusLaneBoundaries.Alignment = -1;
    BusLaneBoundaries.PreserveElementDimensions = false;
    saveVarsTmp{1} = Simulink.BusElement;
    saveVarsTmp{1}.Name = 'NumLaneBoundaries';
    saveVarsTmp{1}.Complexity = 'real';
    saveVarsTmp{1}.Dimensions = [1 1];
    saveVarsTmp{1}.DataType = 'double';
    saveVarsTmp{1}.Min = [];
    saveVarsTmp{1}.Max = [];
    saveVarsTmp{1}.DimensionsMode = 'Fixed';
    saveVarsTmp{1}.SamplingMode = 'Sample based';
    saveVarsTmp{1}.DocUnits = '';
    saveVarsTmp{1}.Description = '';
    saveVarsTmp{1}(2, 1) = Simulink.BusElement;
    saveVarsTmp{1}(2, 1).Name = 'Time';
    saveVarsTmp{1}(2, 1).Complexity = 'real';
    saveVarsTmp{1}(2, 1).Dimensions = [1 1];
    saveVarsTmp{1}(2, 1).DataType = 'double';
    saveVarsTmp{1}(2, 1).Min = [];
    saveVarsTmp{1}(2, 1).Max = [];
    saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(2, 1).DocUnits = '';
    saveVarsTmp{1}(2, 1).Description = '';
    saveVarsTmp{1}(3, 1) = Simulink.BusElement;
    saveVarsTmp{1}(3, 1).Name = 'LaneBoundaries';
    saveVarsTmp{1}(3, 1).Complexity = 'real';
    saveVarsTmp{1}(3, 1).Dimensions = [2 1];
    saveVarsTmp{1}(3, 1).DataType = 'Bus: BusLaneBoundariesLaneBoundaries';
    saveVarsTmp{1}(3, 1).Min = [];
    saveVarsTmp{1}(3, 1).Max = [];
    saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(3, 1).DocUnits = '';
    saveVarsTmp{1}(3, 1).Description = '';
    BusLaneBoundaries.Elements = saveVarsTmp{1};
    clear saveVarsTmp;
    
    BusLaneBoundariesLaneBoundaries = Simulink.Bus;
    BusLaneBoundariesLaneBoundaries.Description = '';
    BusLaneBoundariesLaneBoundaries.DataScope = 'Auto';
    BusLaneBoundariesLaneBoundaries.HeaderFile = '';
    BusLaneBoundariesLaneBoundaries.Alignment = -1;
    BusLaneBoundariesLaneBoundaries.PreserveElementDimensions = false;
    saveVarsTmp{1} = Simulink.BusElement;
    saveVarsTmp{1}.Name = 'Coordinates';
    saveVarsTmp{1}.Complexity = 'real';
    saveVarsTmp{1}.Dimensions = [nvp.laneDimensions 3];
    saveVarsTmp{1}.DataType = 'double';
    saveVarsTmp{1}.Min = [];
    saveVarsTmp{1}.Max = [];
    saveVarsTmp{1}.DimensionsMode = 'Fixed';
    saveVarsTmp{1}.SamplingMode = 'Sample based';
    saveVarsTmp{1}.DocUnits = '';
    saveVarsTmp{1}.Description = '';
    saveVarsTmp{1}(2, 1) = Simulink.BusElement;
    saveVarsTmp{1}(2, 1).Name = 'Curvature';
    saveVarsTmp{1}(2, 1).Complexity = 'real';
    saveVarsTmp{1}(2, 1).Dimensions = [nvp.laneDimensions 1];
    saveVarsTmp{1}(2, 1).DataType = 'double';
    saveVarsTmp{1}(2, 1).Min = [];
    saveVarsTmp{1}(2, 1).Max = [];
    saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(2, 1).DocUnits = '';
    saveVarsTmp{1}(2, 1).Description = '';
    saveVarsTmp{1}(3, 1) = Simulink.BusElement;
    saveVarsTmp{1}(3, 1).Name = 'CurvatureDerivative';
    saveVarsTmp{1}(3, 1).Complexity = 'real';
    saveVarsTmp{1}(3, 1).Dimensions = [nvp.laneDimensions 1];
    saveVarsTmp{1}(3, 1).DataType = 'double';
    saveVarsTmp{1}(3, 1).Min = [];
    saveVarsTmp{1}(3, 1).Max = [];
    saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(3, 1).DocUnits = '';
    saveVarsTmp{1}(3, 1).Description = '';
    saveVarsTmp{1}(4, 1) = Simulink.BusElement;
    saveVarsTmp{1}(4, 1).Name = 'HeadingAngle';
    saveVarsTmp{1}(4, 1).Complexity = 'real';
    saveVarsTmp{1}(4, 1).Dimensions = [1 1];
    saveVarsTmp{1}(4, 1).DataType = 'double';
    saveVarsTmp{1}(4, 1).Min = [];
    saveVarsTmp{1}(4, 1).Max = [];
    saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(4, 1).DocUnits = '';
    saveVarsTmp{1}(4, 1).Description = '';
    saveVarsTmp{1}(5, 1) = Simulink.BusElement;
    saveVarsTmp{1}(5, 1).Name = 'LateralOffset';
    saveVarsTmp{1}(5, 1).Complexity = 'real';
    saveVarsTmp{1}(5, 1).Dimensions = [1 1];
    saveVarsTmp{1}(5, 1).DataType = 'double';
    saveVarsTmp{1}(5, 1).Min = [];
    saveVarsTmp{1}(5, 1).Max = [];
    saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(5, 1).DocUnits = '';
    saveVarsTmp{1}(5, 1).Description = '';
    saveVarsTmp{1}(6, 1) = Simulink.BusElement;
    saveVarsTmp{1}(6, 1).Name = 'BoundaryType';
    saveVarsTmp{1}(6, 1).Complexity = 'real';
    saveVarsTmp{1}(6, 1).Dimensions = [1 1];
    saveVarsTmp{1}(6, 1).DataType = 'uint8';
    saveVarsTmp{1}(6, 1).Min = [];
    saveVarsTmp{1}(6, 1).Max = [];
    saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(6, 1).DocUnits = '';
    saveVarsTmp{1}(6, 1).Description = '';
    saveVarsTmp{1}(7, 1) = Simulink.BusElement;
    saveVarsTmp{1}(7, 1).Name = 'Strength';
    saveVarsTmp{1}(7, 1).Complexity = 'real';
    saveVarsTmp{1}(7, 1).Dimensions = [1 1];
    saveVarsTmp{1}(7, 1).DataType = 'double';
    saveVarsTmp{1}(7, 1).Min = [];
    saveVarsTmp{1}(7, 1).Max = [];
    saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(7, 1).DocUnits = '';
    saveVarsTmp{1}(7, 1).Description = '';
    saveVarsTmp{1}(8, 1) = Simulink.BusElement;
    saveVarsTmp{1}(8, 1).Name = 'Width';
    saveVarsTmp{1}(8, 1).Complexity = 'real';
    saveVarsTmp{1}(8, 1).Dimensions = [1 1];
    saveVarsTmp{1}(8, 1).DataType = 'double';
    saveVarsTmp{1}(8, 1).Min = [];
    saveVarsTmp{1}(8, 1).Max = [];
    saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(8, 1).DocUnits = '';
    saveVarsTmp{1}(8, 1).Description = '';
    saveVarsTmp{1}(9, 1) = Simulink.BusElement;
    saveVarsTmp{1}(9, 1).Name = 'Length';
    saveVarsTmp{1}(9, 1).Complexity = 'real';
    saveVarsTmp{1}(9, 1).Dimensions = [1 1];
    saveVarsTmp{1}(9, 1).DataType = 'double';
    saveVarsTmp{1}(9, 1).Min = [];
    saveVarsTmp{1}(9, 1).Max = [];
    saveVarsTmp{1}(9, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(9, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(9, 1).DocUnits = '';
    saveVarsTmp{1}(9, 1).Description = '';
    saveVarsTmp{1}(10, 1) = Simulink.BusElement;
    saveVarsTmp{1}(10, 1).Name = 'Space';
    saveVarsTmp{1}(10, 1).Complexity = 'real';
    saveVarsTmp{1}(10, 1).Dimensions = [1 1];
    saveVarsTmp{1}(10, 1).DataType = 'double';
    saveVarsTmp{1}(10, 1).Min = [];
    saveVarsTmp{1}(10, 1).Max = [];
    saveVarsTmp{1}(10, 1).DimensionsMode = 'Fixed';
    saveVarsTmp{1}(10, 1).SamplingMode = 'Sample based';
    saveVarsTmp{1}(10, 1).DocUnits = '';
    saveVarsTmp{1}(10, 1).Description = '';
    BusLaneBoundariesLaneBoundaries.Elements = saveVarsTmp{1};
    clear saveVarsTmp;

    assignin('base','BusLaneBoundariesLaneBoundaries',BusLaneBoundariesLaneBoundaries)
    assignin('base','BusLaneBoundaries',BusLaneBoundaries)
end