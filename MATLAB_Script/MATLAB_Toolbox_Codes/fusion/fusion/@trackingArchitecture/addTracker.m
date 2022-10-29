function out = addTracker(obj, tracker, varargin)
%addTracker  Add a tracker to the tracking architecture.
% addTracker(TA,TRACKER) adds the tracker, TRACKER, to the
% trackingArchitecture, TA. Use this syntax if TRACKER implements the
% sensorIndices method. For example, trackerPHD implements the
% sensorIndices method.
%
% addTracker(TA,TRACKER,'SensorIndices',INDICES) adds the tracker, TRACKER,
% to the trackingArchitecture, TA, with the sensor indices defined in
% INDICES as the sensors reporting objectDetections to this tracker. Use
% this syntax if TRACKER does not implement the sensorIndices method. For
% example, trackerGNN, trackerJPDA, and trackerTOMHT do not implement this
% method.
%
% addTracker(..., 'ToOutput', FLAG), additionally, allows you to define if
% the output of this tracker appears in the output of the tracking
% architecture step. FLAG is either true or false.
%
% addTracker(..., 'Name', NAME), additionally, allows you to specify a name
% for the tracker. This name is shown in the summary and show methods.
%
% ARCH = addTracker(...), additionally returns the summary of the tracking
% architecture after adding the tracker.
%
% See also: trackingArchitecture/Summary trackingArchitecture/addTrackFuser

% Copyright 2020 The MathWorks, Inc.

validateATracker(tracker);
requiresIndices = ~ismethod(tracker,'sensorIndices');
ind = fusion.internal.findProp('SensorIndices',varargin{:});
hasIndices = ind < numel(varargin);
coder.internal.errorIf(requiresIndices && ~hasIndices, 'fusion:trackingArchitecture:trackerRequiresIndices','sensorIndices','SensorIndices');
coder.internal.errorIf(~requiresIndices && hasIndices, 'fusion:trackingArchitecture:trackerImplementsSensorIndices','sensorIndices','SensorIndices');
if hasIndices
    argin = {varargin{ind+1},varargin{1:ind-1},varargin{ind+2:end}};
else
    sensInds = sensorIndices(tracker);
    argin = [{sensInds}, varargin(:)'];
end
addNode(obj, tracker, argin{:});
obj.pNumTrackers = obj.pNumTrackers + 1;
out = summary(obj);
end

function validateATracker(value)
cond = isa(value,'matlabshared.tracking.internal.fusion.AbstractTracker') || ...
    isa(value,'fusion.trackingArchitecture.Tracker');
coder.internal.assert(cond,'fusion:trackingArchitecture:expectedTracker','TRACKER','fusion.trackingArchitecture.Tracker');
end