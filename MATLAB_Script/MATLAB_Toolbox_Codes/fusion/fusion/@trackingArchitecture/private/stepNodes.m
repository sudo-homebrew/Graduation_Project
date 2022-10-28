function stepNodes(obj, nodesToRun, dets, inTrks, t)
%stepNodes  Step each node in the trackingArchitecture
%  nodesToRun is a logical array of nodes to run in this step
%  dets is an array of detections from the trackingArchitecture input
%  inTrks is an array of tracks from the trackingRachitecture input
%  t is time

% Copyright 2020 The MathWorks, Inc.

% Reset the current step tracks
obj.pCurrentStepTracks(:) = {objectTrack.empty};

for level = obj.pUniqueLevels
    toRunThisLevel = find(level == obj.pNodeLevels & nodesToRun);
    for nodeInd = toRunThisLevel
        node = obj.pNodes{nodeInd};
        if node.IsTracker
            obj.pCurrentStepTracks{nodeInd} = step(node,dets,t);
        elseif node.IsFuser
            tracksToNode = collectTracksToNode(obj,node,inTrks);
            obj.pCurrentStepTracks{nodeInd} = step(node,tracksToNode,t);
        end
    end
end
end

function tracksToNode = collectTracksToNode(obj,node,inTrks)
t = [obj.pCurrentStepTracks(:)',{inTrks}];
if isempty(obj.pDelayedEdges)
    tracksToNode = vertcat(t{:});
    return
end
sourceIDs = node.Inputs;
thisID = node.Index;
for id = sourceIDs
    if any(all([id thisID]==obj.pDelayedEdges,2),1)
        thisInd = (id==obj.pNodeIndices);
        t{thisInd} = obj.pPrevStepTracks{thisInd};
    end
end
tracksToNode = vertcat(t{:});
end