function varargout = show(obj,varargin)
%SHOW  Plot the tracking architecture in a figure
%  SHOW(ta) shows the tracking architecture in a new figure.
%  ha = SHOW(ta) shows the tracking architecture and returns a
%  handle to the axes.
%
%  SHOW(...,'Parent',AX), additionally, allows you to provide the axes, AX,
%  in which the architecture is shown.
%
%  % Example: Create and show a trackingArchitecture in a plot.
%  % This architecture has two trackers:
%  % Tracker 1: GNN that receives detections from sensors 1 and 2.
%  % Tracker 2: JPDA that receives detections from sensor 3.
%  % Both trackers pass tracks to a trackFuser that also receives tracks
%  % directly from a tracking sensor, 4.
%  arch = trackingArchitecture;
%  addTracker(arch, trackerGNN('TrackerIndex',1),'SensorIndices',[1 2]);
%  addTracker(arch, trackerJPDA('TrackerIndex',2),'SensorIndices',3);
%  fuser = trackFuser('FuserIndex',3,'MaxNumSources',3,...
%     'SourceConfigurations',{fuserSourceConfiguration(1);...
%     fuserSourceConfiguration(2);fuserSourceConfiguration(4)});
%  addTrackFuser(arch,fuser);
%
%  % View the architecture in a plot
%  show(arch)

%  Copyright 2020-2021 The MathWorks, Inc.

narginchk(1,3)
if nargin==2
    error(message('fusion:trackingArchitecture:invalidNarginShow','show',1,3));
end
% Prepare the figure
if isempty(varargin)
    ha = newplot;
    f = ha.Parent;
    n = num2str(get(f,'Number'));
    set(f,'Tag',strcat('taFig',n));
    set(ha,'Tag',strcat('taAx',n));
else
    validatestring(varargin{1},{'Parent'},'show');
    validateattributes(varargin{2},{'matlab.graphics.axis.Axes'},{'scalar'},'show','AX');
    ha = varargin{2};
end
ha.XAxis.Visible = "off";
ha.YAxis.Visible = "off";
axis equal
hold(ha,'on'); cl = onCleanup(@() hold(ha,'off'));

if ~isempty(obj.ArchitectureName)
    %If ArchitectureName property is not defined use input name.
    archTitle = obj.ArchitectureName;
else
    archTitle = char(inputname(1));
end
title(ha,strcat(getString(message('fusion:trackingArchitecture:plotTitle')),": ",archTitle));
varargout = cell(1,nargout);
[varargout{1:nargout}] = ha;

% Analyze architecture topology 
[nodePos,edgeNodes,delayedEdges] = nodePositions(obj);
if isempty(nodePos) % Empty architcture
    return
end
allXs = [nodePos.XPosition];
allYs = [nodePos.YPosition];
ha.XLim = [min(allXs)-0.5, max(allXs)+1];
ha.YLim = [min(allYs)-0.5, max(allYs)+0.5];
s = summary(obj);

% Position nodes and architecture outputs
boxHeight = 0.33;
boxWidth = 0.5;
allNodeIndices = cellfun(@(c) uint32(c.Index), obj.pNodes);
numBoxes = numel(nodePos);
inPts = zeros(numBoxes,2);
outPts = zeros(numBoxes,2);
node2node = zeros(numBoxes,1);
for i = 1:numel(nodePos)
    if nodePos(i).IsNode
        noden = char(nodePos(i).Name);
        nodeIndex = str2double(noden(2:end));
        if ~isnan(nodeIndex) % regular node
            idx = nodeIndex == allNodeIndices;
            node2node(nodeIndex) = i;
            node = obj.pNodes{idx};
            [inPts(i,:),outPts(i,:)] = addNodeToPlot(node,nodePos(i).XPosition,nodePos(i).YPosition,boxWidth,boxHeight);
            if node.ToOutput
                addOutputFromNode(s.ArchitectureOutput(idx),outPts(i,:))
            end
        end
    else
        outPts(i,:) = addInputPort(nodePos(i));
    end
end

% Connect regular edges
hasRegularEdge = false(size(outPts,1),1);
edgeCenterLine = zeros(size(edgeNodes,1),3);
for i = 1:size(edgeNodes,1)
    tag = mat2str(edgeNodes(i,:));
    sourcePt = outPts(edgeNodes(i,1),:);
    targetPt = inPts(edgeNodes(i,2),:);
    [x,y] = directEdgePositions(sourcePt,targetPt);
    edgeCenterLine(i,:) = [x(2),y(2),y(3)];
    [x,y] = deconflictEdges(edgeCenterLine,i,x,y);
    arrow(x,y,'Color','k','Tag',tag);
    hasRegularEdge(edgeNodes(i,1)) = true;
end

% If there are delayed edges, replace each source node with a delay node
% from that source and replace the outPts with the delay node
alreadyHasDelayNode = false(1,size(outPts,1));
for i = 1:size(delayedEdges,1)
    idx1 = node2node(delayedEdges(i,1));
    if ~alreadyHasDelayNode(idx1)
        tag = string(delayedEdges(i,1));
        [~,outPts(idx1,:)] = addDelayNode(outPts(idx1,1),outPts(idx1,2),tag,hasRegularEdge(idx1));
        alreadyHasDelayNode(idx1) = true;
    end
end
    
% Connect delayed edges
for i = 1:size(delayedEdges,1)
    idx1 = node2node(delayedEdges(i,1));
    idx2 = node2node(delayedEdges(i,2));
    tag = mat2str(delayedEdges(i,:));
    sourcePt = outPts(idx1,:);
    targetPt = inPts(idx2,:);
    [x,y] = delayedEdgePositions(sourcePt,targetPt);
    arrow(x,y,'Color','k','Tag',tag)
end
end

function [inpoint,outpoint] = addNodeToPlot(node,i,j,boxWidth,boxHeight)
s = summary(node);
tag = strcat("NODE",s.System);
rectangle('pos', [i-boxWidth/2,j-boxHeight/2,boxWidth,boxHeight],'Tag',tag);
boxedText(i-boxWidth/2+0.02,j,strrep(s.System,":",newline),boxWidth,'Tag',strcat('TEXT',tag));
inpoint = [i-boxWidth/2 j];
outpoint = [i+boxWidth/2 j];
end

function addOutputFromNode(s,outPts)
if iscell(s)
    s = s{1};
end
idstr = num2str(s);
tag = strcat('OUTPORT',idstr);
rectangle('Position', [outPts(1)+0.1,outPts(2)+0.05,0.12,0.08],'Curvature',[1 1],'Tag',tag);
boxedText(outPts(1)+0.15,outPts(2)+0.09,idstr,0.12,'Tag',strcat('TEXT',tag))
arrow(outPts(1)+[0;0.05;0.05;0.1],outPts(2)+[0;0;+0.1;+0.1],'Color','k','Tag',strcat('OUTARROW',idstr))
end

function outPt = addInputPort(nodePos)
name = nodePos.Name;
tag = strcat('INPORT',name);
rectangle('Position', [nodePos.XPosition,nodePos.YPosition-0.04,0.12,0.08],'Curvature',[1 1],'Tag',tag);
boxedText(nodePos.XPosition-0.05,nodePos.YPosition-0.08,name,0.2,'Tag',strcat('TEXT',tag))
if contains(name,"Detections")
    str = extractAfter(name,10);
else
    str = extractAfter(name,6);
end
boxedText(nodePos.XPosition+0.05,nodePos.YPosition,str,0.12,'Tag',strcat('INTEXT',tag));
outPt = [nodePos.XPosition+0.12,nodePos.YPosition];
end

function [inpoint,outpoint] = addDelayNode(xPos,yPos,tag,hasRegularNode)
if hasRegularNode 
    shiftx = 0.1;
    shifty = 0.25;
else
    shiftx = 0.25;
    shifty = -0.05;
end
rectangle('Position',[xPos+shiftx,yPos+shifty,0.1,0.1],'Tag',strcat('DELAYBOX',tag));
boxedText(xPos+shiftx+0.025,yPos+shifty+0.05,'z^{-1}',0.2,'Tag',strcat('DELAYTEXT',tag));
x = xPos + [0,shiftx/2,shiftx/2,shiftx];
y = yPos + [0,0,shifty+0.05,shifty+0.05];
arrow(x,y,'Color','k');
inpoint = [xPos+shiftx,yPos+shifty+0.05];
outpoint = [xPos+shiftx+0.1,yPos+shifty+0.05];
end

function arrow(x,y,varargin)
line(x,y,varargin{:});
arrowhead = 0.03 * [-1 -0.7;-1 0.7;0 0;-1 -0.7];
angle = atan2(y(end)-y(end-1),x(end)-x(end-1));
arrowheadx = arrowhead(:,1) * cos(angle) - arrowhead(:,2) * sin(angle);
arrowheady = arrowhead(:,1) * sin(angle) + arrowhead(:,2) * cos(angle);
a = [x(end),y(end)] + [arrowheadx arrowheady];
fill(a(:,1),a(:,2),'k');
end

function boxedText(x,y,t,boxSize,varargin)
th = text(x,y,t, 'Clipping', 'On', varargin{:});
if th.Extent(4) > boxSize
    th.FontSize = max(round(th.FontSize * 0.9 * boxSize / th.Extent(4)),8);
end
end

function [x,y] = directEdgePositions(sourcePt,targetPt)
dx = floor(abs(sourcePt(2)-targetPt(2)))/30;
x = [sourcePt(1);targetPt(1)-0.15+dx;targetPt(1)-0.15+dx;targetPt(1)];
dy = (sourcePt(2)-targetPt(2))/30;
y = [sourcePt(2);sourcePt(2);targetPt(2)+dy;targetPt(2)+dy];

end

function [x,y] = delayedEdgePositions(sourcePt,targetPt)
e = 0.1*rand;
dy = min(max(targetPt(2)-sourcePt(2),-5),5);
dx = targetPt(1)-sourcePt(1);
% These edges can go "forward" or "backward"
if dx<0 % Backward
    minDepartShiftX = max(0.01,0.05-dy/25+e/3); % Guarantees to always leave the delay node from right to left
    minArrivalShiftX = max(0.03,0.22-e*abs(dy)/2); % Guaratees to always arrive at the next node from right to left
    maxShiftY = min(0.15,max(e*dy/1.25,-0.15));
    x = [sourcePt(1);sourcePt(1)+minDepartShiftX;sourcePt(1)+minDepartShiftX;targetPt(1)-minArrivalShiftX;targetPt(1)-minArrivalShiftX;targetPt(1)];
    y = [sourcePt(2);sourcePt(2);targetPt(2)-0.35*sign(dy)+e*1.8;targetPt(2)-0.35*sign(dy)+e*1.8;targetPt(2)-maxShiftY;targetPt(2)-maxShiftY];
else
    x = [sourcePt(1);sourcePt(1)+0.1-dy/25+e;sourcePt(1)+0.1-dy/25+e;targetPt(1)-0.2+e*abs(dy)/2;targetPt(1)-0.2+e*abs(dy)/2;targetPt(1)];
    y = [sourcePt(2);sourcePt(2);sourcePt(2);sourcePt(2);targetPt(2)-dy/30;targetPt(2)-dy/30];
end
end

function [x,y] = deconflictEdges(edgeCenterLine,i,x,y)
% Tries to deconflict lines that might be on top of each other with the
% same x value and a similar range of y values.
sameX = find(edgeCenterLine(i,1) == edgeCenterLine(1:i-1,1));
minthisy = min(edgeCenterLine(i,2:3));
maxthisy = max(edgeCenterLine(i,2:3));
for ind = 1:numel(sameX)
    minthaty = min(edgeCenterLine(sameX(ind),2:3));
    maxthaty = max(edgeCenterLine(sameX(ind),2:3));
    % The lines intersect (and thus are on top of each other partially) if
    % one end falls with the range of the other line segment
    ontop = (minthisy > minthaty && minthisy < maxthaty) || ...
        (maxthisy > minthaty && maxthisy < maxthaty) || ...
        (minthaty > minthisy && minthaty < maxthisy) || ...
        (maxthaty > minthisy && maxthaty < maxthisy);
    if ontop
        dx = -0.05*rand-0.01;
        x(2) = x(2)+dx;
        x(3) = x(3)+dx;
    end
end
end