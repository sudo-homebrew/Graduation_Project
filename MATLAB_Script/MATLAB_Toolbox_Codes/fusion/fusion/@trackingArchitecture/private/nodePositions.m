function [nodePos,edges,delayedEdges] = nodePositions(obj)
%nodePositions  Calculate the position of each node for display
% nodePos = nodePositions(obj) provides the position of each node on a
% grid. Columns should be organized left-to-right according to this order:
%   1. All detection input "ports".
%   2. All trackers.
%   3 and onwards. All fusers starting from topology level 2 and on.
%
% See also: trackingArchitecture/analyzeTopology

% Copyright 2020 The Mathworks, Inc.

% Allocate memory
[~,delayedEdges,levels,orderedNodes] = analyzeTopology(obj);

% Node indices get a high value
nodeIndices = cellfun(@(c) uint32(c.Index), obj.pNodes);
nodeNames = repmat("a",numel(nodeIndices),1);

% Create all the edges needed
s = string.empty(1,0);
t = string.empty(1,0);
for i = 1:numel(obj.pNodes)
    inputs = obj.pNodes{i}.Inputs;
    if obj.pIsTracker(i)
        s(end+1:end+numel(inputs)) = "Detections" + inputs;
        nodeNames(i) = "T" + nodeIndices(i);
    elseif obj.pIsFuser(i)
        inputFromNodes = ismember(inputs,nodeIndices');
        intoNodes = find(any(inputs == nodeIndices',2));
        inputIsTracker = obj.pIsTracker(intoNodes);
        inputIsFuser = obj.pIsFuser(intoNodes);
        s(end+1:end+numel(inputs)) = ["T" + nodeIndices(intoNodes(inputIsTracker)),...
            "F" + nodeIndices(intoNodes(inputIsFuser)), "Tracks" + inputs(~inputFromNodes)];
        nodeNames(i) = "F" + nodeIndices(i);
    end
    t(end+1:end+numel(inputs)) = nodeNames(i);
end

% Get rid of cycles in the graph
d = "F" + delayedEdges;
for i = 1:size(d,1)
    inLoop = (s==d(i,1) & t==d(i,2));
    s = s(~inLoop);
    t = t(~inLoop);
end
g = digraph(s,t);
nNodes = numnodes(g);
nNames = [g.Nodes{:,1}];
nodeCoords = layeredLayout(MLDigraph(g), [], [], 'auto');
nodeCoords = max(nodeCoords) - nodeCoords + 1;
nodePos = repmat(struct('Name'," ",'IsNode',false,'XPosition', 0, 'YPosition', 0),nNodes,1);
for i = 1:size(nodeCoords,1)
    nodePos(i) = setNodePos(nNames{i},nodeCoords(i,:));
end
edges = extractEdges(g);

% Add back the nodes that can only be accessed by delayed edges
missingNodes = setdiff(nodeNames,nNames);
if ~isempty(missingNodes)
    nodePos = addMissingNodes(nodePos,nNames,missingNodes,delayedEdges,levels,orderedNodes);
end
end

function s = setNodePos(name,coord)
s.Name = string(name);
s.XPosition = coord(2); % Turn y to x
s.YPosition = coord(1); % Turn x to y
s.IsNode = ~contains(name,["Detections","Tracks"]);
end

function edges = extractEdges(g)
e = g.Edges;
endNodes = e.EndNodes;
edges = reshape(findnode(g,endNodes),[],2);
end

function nodePos = addMissingNodes(nodePos,nNames,missingNodes,delayedEdges,levels,orderedNodes)
numNodes = numel(nodePos);
numMissingNodes = numel(missingNodes);
nodePos = [nodePos;repmat(nodePos(1),numMissingNodes,1)];
nNames = vertcat(nNames,cell(numMissingNodes,1));
for i = 1:numMissingNodes
    mn = char(missingNodes{i});
    thisNode = str2double(mn(2:end));
    leadingEdges = (delayedEdges(:,2)==thisNode);
    leadingNodes = delayedEdges(leadingEdges,1);
    leadingNodeNames = "F"+leadingNodes;
    level = 0;
    for j = 1:numel(leadingNodes)
        inOrder = (leadingNodes(j)==orderedNodes);
        if levels(inOrder) > level
            level = levels(inOrder);
            afterPos = find(strcmpi(leadingNodeNames(j),nNames));
            levels(thisNode==orderedNodes) = level+1;
            posX = nodePos(afterPos).XPosition+1.25;
            posY = nodePos(afterPos).YPosition;
        elseif levels(inOrder) == level
            afterPos = strcmpi(leadingNodeNames(j),nNames);
            posY = (posY + nodePos(afterPos).YPosition)/2;
        end
    end
    nodePos(numNodes+i) = setNodePos(missingNodes{i},[posY,posX]);
    nNames{numNodes+i} = missingNodes{i};
end
end