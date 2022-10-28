function model = exportToSimulink(obj,args)
% EXPORTTOSIMULINK Export the tracking architecture to a Simulink model.
%
% EXPORTTOSIMULINK(TA) exports the tracking architecture, TA, as a
% subsystem in a new Simulink model.
%
% EXPORTTOSIMULINK(..., 'Model', MODEL), additionally, allows you to export
% the tracking architecture to an existing Simulink model, MODEL. MODEL can
% be the name or handle of the Simulink model. If a Simulink model
% with name MODEL does not exist, a new model is created.
%
% model = EXPORTTOSIMULINK(TA, ...) exports the tracking architecture, TA,
% to a Simulink model and returns the model name.
%
%  % Example: Create and export a trackingArchitecture in a Simulink model.
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
%
%  % Export the architecture to a Simulink model
%  exportToSimulink(arch, 'Model', 'ArchitectureModel');
%
% See also: trackingArchitecture/summary trackingArchitecture/show

% Copyright 2021 The MathWorks, Inc.

arguments
    obj(1,1) trackingArchitecture
    args.Model(1,:) {mustBeCharOrNumeric} = char.empty
end

if isempty(args.Model)
    model = fusion.simulink.internal.initializeSimulinkModel();
else
    model = fusion.simulink.internal.initializeSimulinkModel(args.Model);
end

if isempty(obj.ArchitectureName)
    %If ArchitectureName property is not defined use input name.
    ArchitectureName = inputname(1);
else
    ArchitectureName = obj.ArchitectureName;
end

%Architecture summary
summary = obj.summary;
%Nodes within the architecture.
nodes = obj.pNodes;
numNodes = numel(nodes);

%Preallocate memory to store some meta data.
hblocks = zeros(numNodes,1);
hConcatBlocks = zeros(numNodes,1);
hDelayBlocks = zeros(numNodes,1);
addedNodes = cell(numNodes,1);

%Analyze topology and calculate node positions
[nodePoses,~,delayedEdges] = nodePositions(obj);
maxYPos = max([nodePoses.YPosition]);
%In Simulink, origin is top left corner and calculated positions assume
%origin at bottom left corner. Converting the calculated node position
%in Simulink coordinate system.
for i = 1:numel(nodePoses)
    nodePoses(i).YPosition = maxYPos - nodePoses(i).YPosition;
end
[verticalGap,horizontalGap,defaultHeight,defaultWidth] = ...
    fusion.simulink.internal.trackingArchitecture.calculatelGapsAndDefaultBlockSize();

% Create a tracking sub-system
trackingSubsystem = fusion.simulink.internal.trackingArchitecture.createSubsystem(...
    model,ArchitectureName,nodes);

%Add tracker and fuser blocks in the model
for i = 1:numNodes
    numInputs = numel(nodes{i}.Inputs);
    if nodes{i}.IsTracker
        % node name here is same as the name shown in summary.
        nodeName = ['T', num2str(nodes{i}.Index),':',nodes{i}.Name];
        %Store the name of added node.
        addedNodes =  fusion.simulink.internal.trackingArchitecture.getSetAddedNodeName(...
            nodes,addedNodes,nodes{i}.Index,nodeName);
        %Add an equivalent block to the model and store the handle to
        %the block.
        hblocks(i) = fusion.simulink.internal.trackingArchitecture.addNodeToModel(...
            nodes{i}.ManagedNode,trackingSubsystem,nodeName,defaultHeight,defaultWidth);
        %Find current node's position
        thisNode = nodePoses([nodePoses.Name] == ['T', num2str(nodes{i}.Index)]);
        %Arrange the tracker block in the model with the positions
        %calculated by the trackingArchitecture.
        fusion.simulink.internal.trackingArchitecture.positionBlock(...
            hblocks(i),(thisNode.XPosition),thisNode.YPosition*verticalGap);

        trackerHandle = hblocks(i);
        if numInputs > 1
            %Add a DetectionConcatenation block if there are more than
            %one inputs to the tracker and store the handle.
            hConcatBlocks(i) = fusion.simulink.internal.trackingArchitecture.addConcatenationBlock(...
                trackingSubsystem,nodeName,hblocks(i),numInputs);
            trackerHandle = hConcatBlocks(i);
        end
        for j = 1:numInputs
            %For each input to the tracker add an inport in the model.
            name = ['Detections',num2str(nodes{i}.Inputs(j))];
            fusion.simulink.internal.trackingArchitecture.addInport(...
                trackingSubsystem,trackerHandle,name);
        end
        %Position inports in the model.
        fusion.simulink.internal.trackingArchitecture.positionInports(...
            trackerHandle,numInputs);
    else
        %node is a fuser
        % Fuser is a special node, as it can receive and send tracks to
        % and from other nodes in architecture. When we export a
        % fuser node, we follow 3 steps:
        % 1. Add the fuser node.
        % 2. Analyze its outputs and connect to other nodes.
        % 3. Analyze its inputs and connect to other nodes.

        %Step 1: Add the fuser node.

        % node name here is same as the name shown in summary.
        nodeName = ['F', num2str(nodes{i}.Index),':',nodes{i}.Name];
        %Store the name of the added node.
        addedNodes = fusion.simulink.internal.trackingArchitecture.getSetAddedNodeName(...
            nodes,addedNodes,nodes{i}.Index,nodeName);
        %Find the current node position.
        thisNode = nodePoses([nodePoses.Name] == ['F', num2str(nodes{i}.Index)]);
        %Add am equivalent fuser block in the model and store the
        %handle.
        hblocks(i) = fusion.simulink.internal.trackingArchitecture.addNodeToModel(...
            nodes{i}.ManagedNode,trackingSubsystem,nodeName,defaultHeight,defaultWidth);
        fuserHandle = hblocks(i);

        %Arrange the fuser block in the model with the positions
        %calculated by the trackingArchitecture.
        fusion.simulink.internal.trackingArchitecture.positionFuserNode(...
            hblocks(i),thisNode,verticalGap,horizontalGap);

        %Step 2: Analyze fuser outputs and make connections.

        currentNodeIdx = nodes{i}.Index;
        %Find all fuser nodes in the architecture.
        fuserNodes = [nodes{cell2mat(cellfun(@(x)x.IsFuser,nodes,'UniformOutput',false))}];
        %Find other fuser nodes connected with current fuser node.
        connectedFuserNodes =  fuserNodes(arrayfun(@(x)any(ismember(x.Inputs,currentNodeIdx)),fuserNodes));
        for k = 1:numel(connectedFuserNodes)
            connectedNodeIdx = connectedFuserNodes(k).Index;
            %Check if the connection is delayed between connected fuser node and current fuser node.
            delayed =  fusion.simulink.internal.trackingArchitecture.isConnectionDelayed(...
                delayedEdges,currentNodeIdx,connectedNodeIdx);
            if delayed
                %Connection is delayed, a unit delay block must be
                %added between current node and connected node.
                blockExists = hDelayBlocks(i);
                if ~blockExists
                    % A delay block for the current fuser node does
                    % not exist. Add a new block.
                    hDelayBlocks(i) = fusion.simulink.internal.trackingArchitecture.addUnitDelayBlock(...
                        trackingSubsystem,currentNodeIdx,hblocks(i),thisNode);
                end
                fuserHandle = hDelayBlocks(i);
            end

            %Connect the fuser/delay block to the connected node. If
            %the connected fuser node is already added in the model,
            %just make the connection otherwise the connection will be
            % made when the connected node is being added.
            if ~isempty(fusion.simulink.internal.trackingArchitecture.getSetAddedNodeName(nodes,addedNodes,connectedNodeIdx))
                %Connected node exists in the model. Find handle to the
                %connected node and make connection.
                h =  fusion.simulink.internal.trackingArchitecture.getNodeHandle(...
                    connectedNodeIdx,hblocks,hConcatBlocks,nodes);
                fusion.simulink.internal.trackingArchitecture.connectBlocks(...
                    trackingSubsystem,fuserHandle,1,h,NaN);
            end
        end

        %Step 3: Analyze fuser inputs and make connections.

        if numInputs>1
            %Add a TrackConcatenation block if there are more than one
            %inputs to the fuser block and store the handle.
            hConcatBlocks(i) = fusion.simulink.internal.trackingArchitecture.addConcatenationBlock(...
                trackingSubsystem,nodeName,hblocks(i),numInputs);
            %Use concatenation block's handle to connect inputs.
            fuserHandle = hConcatBlocks(i);
        else
            %Use fuser block's handle to connect input.
            fuserHandle = hblocks(i);
        end
        %Find architecture input to the fuser node from
        %the architecture summary.
        archInputs = fusion.simulink.internal.trackingArchitecture.getArchitectureInput(summary,i);
        %Iterate over all the inputs to the fuser node. Add an inport
        %if input is an architecture input otherwise connect to the
        %other fuser or tracker nodes in the architecture.
        for j = 1:numInputs
            if any(ismember(archInputs,nodes{i}.Inputs(j)))
                %Input is an Architecture input
                inputName = ['Tracks In',num2str(nodes{i}.Inputs(j))];
                %Add an inport.
                fusion.simulink.internal.trackingArchitecture.addInport(...
                    trackingSubsystem,fuserHandle,inputName);
            else
                %Fuser connects to the trackers or other fusers in the architecture.
                connectedNodeIdx = nodes{i}.Inputs(j);
                isFuser = nodes{cellfun(@(x)x.Index==connectedNodeIdx,nodes)}.IsFuser;
                if isFuser
                    %Connected node is a fuser.

                    %First check if the connected fuser is already added in the
                    %model or not. If the node is added, connect with
                    %the current fuser block. If the node is not added
                    %the connection will be made when the connected
                    %node is being added.
                    if ~isempty(fusion.simulink.internal.trackingArchitecture.getSetAddedNodeName(nodes,addedNodes,connectedNodeIdx))
                        %Check if the connection is delayed between
                        %fuser nodes.
                        delayed = fusion.simulink.internal.trackingArchitecture.isConnectionDelayed(...
                            delayedEdges,connectedNodeIdx,currentNodeIdx);
                        if delayed
                            %Connect to the delay block.
                            h =  fusion.simulink.internal.trackingArchitecture.getNodeHandle(...
                                connectedNodeIdx,hblocks,hDelayBlocks,nodes);
                        else
                            %Connect to the fuser block.
                            h = hblocks(fusion.simulink.internal.trackingArchitecture.getNodeSeqNum(...
                                nodes,connectedNodeIdx));
                        end
                        fusion.simulink.internal.trackingArchitecture.connectBlocks(...
                            trackingSubsystem,h,1,fuserHandle,NaN);
                    end
                else
                    %Fuser is connected to a tracker
                    trackerBlk = hblocks(fusion.simulink.internal.trackingArchitecture.getNodeSeqNum(...
                        nodes,nodes{i}.Inputs(j)));
                    fusion.simulink.internal.trackingArchitecture.connectBlocks(...
                        trackingSubsystem, trackerBlk, 1, fuserHandle, NaN);
                end
            end
        end
        fusion.simulink.internal.trackingArchitecture.positionFuserInports(...
            fuserHandle,nodes{i}.Inputs,archInputs);
    end
    %Node output
    if nodes{i}.ToOutput
        fusion.simulink.internal.trackingArchitecture.addOutport(...
            trackingSubsystem,hblocks(i),summary,i);
    end
end
%Add time ports
fusion.simulink.internal.trackingArchitecture.addTimeInports(...
    trackingSubsystem,nodes,hblocks);

% Fit to window
set_param(trackingSubsystem,'ZoomFactor','FitSystem');

%Open the model
open_system(model);
end

function mustBeCharOrNumeric(model)
if ~(ischar(model) || isstring(model) || isnumeric(model))
    throwAsCaller(MException(message('fusion:simulink:trackingArchitecture:InvalidModel')));
end
end
