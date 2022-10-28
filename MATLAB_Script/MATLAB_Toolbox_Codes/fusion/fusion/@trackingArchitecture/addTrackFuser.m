function out = addTrackFuser(obj, fuser, varargin)
%addTrackFuser  Add a fuser to the tracking architecture.
% addTrackFuser(TA,FUSER) adds the track-to-track fusion object, FUSER, to
% the trackingArchitecture, TA.
%
% addTrackFuser(..., 'ToOutput', FLAG), additionally, allows you to define
% if the output of this fuser appears in the output of the tracking
% architecture step. FLAG is either true or false.
%
% addTrackFuser(..., 'Name', NAME), additionally, allows you to specify a
% name for track fuser. This name is shown in the summary and show methods.
%
% ARCH = addTrackFuser(...), additionally returns the summary of the
% tracking architecture after adding the fuser.
%
% See also: trackingArchitecture/summary trackingArchitecture/addTracker

% Copyright 2020 The MathWorks, Inc.

validateATrackFuser(fuser);
requiresIndices = ~ismethod(fuser,'sourceIndices');
ind = fusion.internal.findProp('SourceIndices',varargin{:});
hasIndices = ind < numel(varargin);
coder.internal.errorIf(requiresIndices && ~hasIndices, 'fusion:trackingArchitecture:fuserRequiresIndices','sourceIndices','SourceIndices');
coder.internal.errorIf(~requiresIndices && hasIndices, 'fusion:trackingArchitecture:fuserImplementsSourceIndices','sourceIndices','SourceIndices');
if hasIndices
    sensInds = varargin{ind+1};
    argin = {varargin{1:ind-1},varargin{ind+2:end}};
else
    sensInds = sourceIndices(fuser);
    argin = varargin;
end
coder.internal.errorIf(any(fuser.FuserIndex==sensInds),'fusion:trackingArchitecture:fuserIsOwnSource','FUSER.FuserIndex','sourceIndices');
addNode(obj, fuser, sensInds, argin{:});
obj.pNumFusers = obj.pNumFusers+1;
out = summary(obj);
end

function validateATrackFuser(value)
cond = isa(value,'fusion.internal.FuserManager') || ...
    isa(value,'fusion.trackingArchitecture.TrackFuser');
coder.internal.assert(cond,'fusion:trackingArchitecture:expectedTrackFuser','FUSER','fusion.trackingArchitecture.TrackFuser');
end