function addNode(obj, varargin)
%addNode  Add a tracker or track-to-track fusion object
% addNode(TA, NODE, SOURCEINDS) adds the tracker or track-to-track fusion
% object, NODE, to the trackingArchitecture, TA, and specifies the indices
% of its sources, SOURCEINDS.
%
% addNode(..., 'UpdateRate', RATE), additionally, lets you specify the
% update rate for this node using the value RATE. If not specified, the
% default is to run the node with every update of the trackingArchitecture.
% If specified, RATE, is the number of updates per second for this node,
% for example, use RATE = 10 to update the node 10 times in a second.
%
% addNode(..., 'ToOutput', FLAG), additionally, lets you specify the tracks
% produced by this node are provided in the trackingArchitecture step
% output.

% Copyright 2020 The MathWorks, Inc.

% Delegate the construction to the node
node = fusion.trackingArchitecture.internal.TrackingNode(varargin{:});
index = node.Index;
coder.internal.errorIf(any(index==obj.pNodeIndices),'fusion:trackingArchitecture:alreadyExists',num2str(index));
obj.pNodes{1,end+1} = node;
obj.pNodeIndices(1,end+1) = index;
obj.pIsFuser(1,end+1) = node.IsFuser;
obj.pIsTracker(1,end+1) = node.IsTracker;
obj.pOutput(1,end+1) = node.ToOutput;
end