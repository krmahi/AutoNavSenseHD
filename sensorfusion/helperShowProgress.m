function helperShowProgress(detectionAssignmentInfo,stopTime)
% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

persistent wb;
if isempty(wb) || ~ishandle(wb)
    wb =  waitbar(0,'Running smoother');
end
waitbar(detectionAssignmentInfo.Time/stopTime,wb);

if detectionAssignmentInfo.Time >= stopTime
    delete(wb);
end
end