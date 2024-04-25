
if ~ispc
    error(['3D Simulation is only supported on Microsoft', ...
        char(174), ' Windows', char(174), '.']);
end

open_system("TLNWithUnrealTestBench");

open_system("TLNWithUnrealTestBench/Sensors and Environment");

%%
xlimit = [-110 70];
ylimit = [-105 105];
hFigure = helperDisplayTrafficLightScene(xlimit, ylimit);
snapnow;
close(hFigure);
%%
open_system("TLNWithUnrealTestBench/Sensors and Environment/" + ...
    "Traffic Light Switching Logic", 'force');

%% Simulate Vehicle Behavior During Green To Red Transition

helperSLTrafficLightNegotiationWithUnrealSetup(...
    "scenario_03_TLN_straight_greenToRed_with_lead_vehicle");

disp(trafficLightConfig');

%%
mpcverbosity('off');
sim("TLNWithUnrealTestBench");

%%
hFigResults = helperPlotTrafficLightControlAndNegotiationResults( ...
    logsout, trafficLightConfig.stateChangeDistance);

%close(hFigResults);

%% Simulate Vehicle Behavior During Red To Green Transition

helperSLTrafficLightNegotiationWithUnrealSetup(...
    "scenario_04_TLN_straight_redToGreen_with_cross_vehicle");

disp(trafficLightConfig');

sim("TLNWithUnrealTestBench");
%%
hFigResults = helperPlotTrafficLightControlAndNegotiationResults( ...
    logsout, trafficLightConfig.stateChangeDistance);
%close(hFigResults);
mpcverbosity('on');
