function checkIfScenarioBuilderIsInstalled()
% This is a tripwire function used to determine if the the scenario builder
% support package is installed or not.

%   Copyright 2023 The MathWorks, Inc.

% Check if support package is installed
breadcrumbFile = 'scenariobuilder.internal.checkIfScenarioBuilderIsInstalled';
try
    % Check if the support package is installed.
    fullpathToUtility = which(breadcrumbFile);
    if isempty(fullpathToUtility)
        % Support package not installed. Display a message and open add-on
        % explorer.
        msg = "Scenario Builder for Automated Driving Toolbox™ support package is not installed." + ...
            " To use this feature/demo you must install the support package from Add-On explorer.";
        
        f = msgbox(msg,'Scenario Builder not installed!',"error");
        waitfor(f)
        % Launch add-on explorer.
        matlab.addons.supportpackage.internal.explorer.showSupportPackages('AD_SCENBUILDER', 'tripwire')
    end
catch e
    throwAsCaller(e);
end
end