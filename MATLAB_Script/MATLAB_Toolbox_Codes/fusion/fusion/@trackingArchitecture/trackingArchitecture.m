classdef trackingArchitecture < matlab.System
    %trackingArchitecture Tracking system-of-system architecture
    % ta = trackingArchitecture creates a tracking architecture definition.
    % The trackingArchitecture allows you to define how a tracking
    % system-of-system is designed from sensor detections to trackers to
    % track fusers.
    %
    % Use the <a href="matlab:help('trackingArchitecture/addTracker')">addTracker</a> and <a href="matlab:help('trackingArchitecture/addTrackFuser')">addTrackFuser</a> methods to add trackers and 
    % fusers to the trackingArchitecture. Use the <a href="matlab:help('trackingArchitecture/summary')">summary</a> method to see a 
    % list of all the trackers and fusers that were added and how they are
    % connected to the architecture inputs, outputs, and to each other.
    %
    % trackingArchitecture Properties:
    %   Trackers            - A cell array of trackers                  (read only)
    %   TrackFusers         - A cell array of track fusers              (read only)
    %   ArchitectureName    - A character array or string
    %
    % Step syntax:
    %   [TRACKS, TRACKS2, ..., TRACKSN] = step(TA, DETS, INTRKS, TIME) runs
    %   the trackingArchitecture, TA. 
    %   DETS is an array or cell array of objectDetection objects or an
    %   array of struct with the same fields as objectDetection. DETS are
    %   routed to the architecture trackers based on their SensorIndex
    %   value and the ArchitectureInputs defined for the tracker.
    %   INTRKS is an array or cell array of objectTrack objects or an array
    %   of struct with the same fields as objectTrack. INTRKS are passed to
    %   the architecture track fusers based on the fusers
    %   SourceConfigurations property.
    %   TIME is the timestamp to which all the trackers and fusers are
    %   stepped.
    %   The outputs TRACKS1, TRACKS2, ..., TRACKSN are all arrays of
    %   confirmed objectTrack objects. The i-th TRACKS output is from the
    %   tracker or track fuser with ArchitectureOutput value i in the
    %   summary method.
    %
    %   System objects may be called directly like a function instead of
    %   using the step method. For example, y = step(obj) and y = obj() are
    %   equivalent.
    %
    % trackingArchitecture Methods:
    %   step                - Run the tracking architecture
    %   addTracker          - Add a tracker to the architecture
    %   addTrackFuser       - Add a track-to-track fusion object to the architecture
    %   summary             - Display a tabular summary of the architecture
    %   show                - Plot the tracking architecture in a figure
    %   exportToSimulink    - Export the architecture to a Simulink model   
    %   release             - Allow property value and input characterstics changes
    %   clone               - Create a copy of the trackingArchitecture
    %   isLocked            - Locked status (logical)
    %   reset               - Resets the states of the trackingArchitecture
    %
    % Example: Create and step a trackingArchitecure:
    % % This architecture has two trackers:
    % % Tracker 1: GNN that receives detections from sensors 1 and 2.
    % % Tracker 2: JPDA that receives detections from sensor 3.
    % % Both trackers pass tracks to a trackFuser that also receives tracks 
    % % directly from a tracking sensor, 4.
    % arch = trackingArchitecture; 
    % addTracker(arch, trackerGNN('TrackerIndex',1),'SensorIndices',[1 2]);
    % addTracker(arch, trackerJPDA('TrackerIndex',2),'SensorIndices',3);
    % fuser = trackFuser('FuserIndex',3,'MaxNumSources',3,...
    %    'SourceConfigurations',{fuserSourceConfiguration(1);...
    %    fuserSourceConfiguration(2);fuserSourceConfiguration(4)});
    % addTrackFuser(arch,fuser);
    %
    % % Review the architecture using the summary method
    % disp(summary(arch))
    %
    % % View the architecture in a plot
    % show(arch)
    %
    % % Create a display to visualize the tracks generated by arch
    % ax=subplot(3,1,1);
    % p1 = theaterPlot('Parent',ax,'XLimits',[-100 150],'YLimits',[-5 15]); 
    % view(2); title('GNN tracks')
    % t1 = trackPlotter(p1,'ConnectHistory','on','ColorizeHistory','on');
    % ax=subplot(3,1,2);
    % p2 = theaterPlot('Parent',ax,'XLimits',[-100 150],'YLimits',[-5 15]); 
    % t2 = trackPlotter(p2,'ConnectHistory','on','ColorizeHistory','on');
    % view(2); title('JPDA tracks')
    % ax=subplot(3,1,3);
    % p3 = theaterPlot('Parent',ax,'XLimits',[-100 150],'YLimits',[-5 15]);
    % t3 = trackPlotter(p3,'ConnectHistory','on','ColorizeHistory','on');
    % view(2); title('Fused tracks')
    %
    % % Step the architecture using detections and tracks saved earlier
    % load('archInputs','detections','tracks');
    % positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0;0 0 0 0 1 0];
    % for i = 1:numel(detections)
    %     [gnnTrks,jpdaTrks,fusedTrks] = arch(detections{i},tracks{i},i);
    %     plotTrack(t1, getTrackPositions(gnnTrks,positionSelector),string([gnnTrks.TrackID]));
    %     plotTrack(t2, getTrackPositions(jpdaTrks,positionSelector),string([jpdaTrks.TrackID]));
    %     plotTrack(t3, getTrackPositions(fusedTrks,positionSelector),string([fusedTrks.TrackID]));
    % end
    %
    % See also: trackerGNN, trackerJPDA, trackerTOMHT, trackerPHD,
    % trackFuser, fusion.trackingArchitecture.Tracker,
    % fusion.trackingArchitecture.TrackFuser, objectDetection, objectTrack
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    properties(Nontunable)
        %ArchitectureName architecture name
        %  A character array or string defining name of the
        %  tracking architecture.
        ArchitectureName(1,:) char = char.empty 
    end

    properties(SetAccess = protected, Dependent)
        %Trackers A cell array of trackers
        %  A list of trackers added to the trackingArchitecture using the
        %  addTracker method.
        %
        %  See also: trackingArchitecture/addTracker
        Trackers
        
        %TrackFusers A cell array of track fusers
        %  A list of trackers added to the trackingArchitecture using the
        %  addTracker method.
        %
        %  See also: trackingArchitecture/addTrackFuser
        TrackFusers
        
    end
    
    properties(Access = {?trackingArchitecture,?matlab.unittest.TestCase}, Dependent)
        %SensorToTracker Routing of sensors to trackers
        %  Define the routing of detections from sensors to trackers as a
        %  N-by-2 array of integer values. The first column is the list of
        %  SensorIndex values and the second column is the list of
        %  TrackerIndex values. For example, the value [1 2] indicates that
        %  detections with SensorIndex = 1 are routed to the tracker with
        %  TrackerIndex = 2.
        SensorToTracker
        
        %TrackToFusers  Routing of Tracks to TrackFusers        (Read Only)
        %  TrackToFusers provides a view of how tracks are routed to the
        %  track-to-track objects listed in TrackFusers. This is a read
        %  only value and depends on the SourceConfigurations property of
        %  each track-to-track object.
        TrackToFusers
        
        %OutputSelection Select which tracks to output
        %  Define the tracks to output as either 'All' or an array of
        %  TrackerIndex and FuserIndex values. The values must all be
        %  integer values and correspond to the indices specified for
        %  trackers and track-to-track objects.
        OutputSelection
    end
    
    % Pre-computed constants
    properties(Access = private)
        pToFuser
        pLocalToFuser
        
    end
    
    properties(Access = protected) % Definitely used
        pPrevStepTracks
        pCurrentStepTracks
        pAreDetectionsValidated = false
        pAreTracksValidated = false
        pNodeLevels
        pUniqueLevels
        pDelayedEdges
    end
    
    properties(Access = {?trackingArchitecture, ?matlab.unittest.TestCase})
        pNumTrackers = 0
        pNumFusers = 0
        pNodes = cell(1,0);
        pNodeIndices = zeros(1,0);
        pIsFuser = false(1,0);
        pIsTracker = false(1,0);
        pOutput = false(1,0);
    end
    
    methods
        function obj = trackingArchitecture(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
        
        function val = get.Trackers(obj)
            val = cellfun(@(c) c.ManagedNode, obj.pNodes, 'UniformOutput', false);
            val = val(obj.pIsTracker);
        end
        
        function val = get.TrackFusers(obj)
            val = cellfun(@(c) c.ManagedNode, obj.pNodes, 'UniformOutput', false);
            val = val(obj.pIsFuser);
        end
        
        function val = get.OutputSelection(obj)
            val = obj.pNodeIndices(obj.pOutput);
        end
        
        function val = get.SensorToTracker(obj)
            val = zeros(0,2);
            for i = 1:numel(obj.pNodes)
                if obj.pIsTracker(i)
                    trackerIndex = obj.pNodeIndices(i);
                    trackerInputs = obj.pNodes{i}.Inputs;
                    val = vertcat(val, [trackerInputs(:), repmat(trackerIndex,numel(trackerInputs),1)]); %#ok<AGROW>
                end
            end
        end
        
        function t2f = get.TrackToFusers(obj)
            t2f = zeros(0,2);
            for i = 1:numel(obj.TrackFusers)
                thisFuser = obj.TrackFusers{i};
                fuserSourceInds = sourceIndices(thisFuser);
                source2Fuser = [fuserSourceInds(:), repmat(thisFuser.FuserIndex, numel(fuserSourceInds),1)];
                t2f = vertcat(t2f, source2Fuser); %#ok<AGROW>
            end
        end
    end
    
    methods(Access=protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.pNumTrackers = numel(obj.Trackers);
            obj.pNumFusers = numel(obj.TrackFusers);
            obj.pCurrentStepTracks = repmat({objectTrack.empty}, 1, obj.pNumTrackers + obj.pNumFusers);
            [~,delayedEdges,level] = analyzeTopology(obj);
            obj.pNodeLevels = level;
            obj.pUniqueLevels = unique(level);
            obj.pDelayedEdges = delayedEdges;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
            for i = 1:numel(obj.pNodes)
                reset(obj.pNodes{i});
            end
        end
        
        function releaseImpl(obj)
            % Initialize / reset discrete-state properties
            
            for i = 1:numel(obj.pNodes)
                release(obj.pNodes{i});
            end
        end

        function validateInputsImpl(obj,detections,tracks,t)
            % Validate inputs to the step method at initialization
            validateDetections(obj,detections);
            validateTracks(obj,tracks);
            validateattributes(t,{'double','single'},{'real','finite',...
                'nonnegative','scalar'},'trackingArchitecture:step','TIME');
        end

        function flag = isInputDataTypeMutableImpl(~,~)
            % Return false if input data type cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(~)
            % Define total number of inputs for system with optional inputs
            num = 3;
        end
        
        function validateDetections(obj,detections)
            % Detections can be one of three things: a cell array of
            % objectDetection objects, a regular array of objectDetection
            % objects, or a struct array with fields similar to
            % objectDetection objects
            
            if isempty(detections) || obj.pAreDetectionsValidated
                return
            end
            
            % Validate only the first cell elements. Assume all others
            % are same class
            if iscell(detections)
                validateattributes(detections{1},{'objectDetection'},{},'trackingArchitecture/step','DETECTIONS')
            elseif isstruct(detections)
                actFns = fieldnames(detections);
                % At the architecture level, we only need SensorIndex
                coder.internal.assert(any(strcmpi('SensorIndex',actFns)),...
                    'fusion:trackingArchitecture:expectedFields','DETECTIONS','objectDetections');
            else % Verify that it is an objectDetection
                validateattributes(detections(1),{'objectDetection'},{},'trackingArchitecture/step','DETECTIONS');
            end
            obj.pAreDetectionsValidated = true;
        end
        
        function validateTracks(obj,tracks)
            % Tracks can be one of three things: a cell array of
            % objectTrack objects, a regular array of objectTrack objects,
            % or a struct array with fields similar to objectTrack objects
            
            if isempty(tracks) || obj.pAreTracksValidated
                return
            end
            
            % Validate only the first cell elements. Assume all others
            % are same class
            if iscell(tracks)
                validateattributes(tracks{1},{'objectTrack'},{},'trackingArchitecture/step','TRACKS')
            elseif isstruct(tracks)
                actFns = fieldnames(tracks);
                % At the architecture level, we only need SourceIndex
                coder.internal.assert(any(strcmpi('SourceIndex',actFns)),...
                    'fusion:trackingArchitecture:expectedFields','TRACKS','objectTrack');
            else % Verify that it is an objectTrack
                validateattributes(tracks(1),{'objectTrack'},{},'trackingArchitecture/step','TRACKS');
            end
            obj.pAreTracksValidated = true;
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            if isnumeric(obj.OutputSelection)
                num = numel(obj.OutputSelection);
            else
                num = numel(obj.Trackers) + numel(obj.TrackFusers);
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties related to object construction
            obj.pNodes = cell(1,numel(s.pNodes));
            for i = 1:numel(s.pNodes)
                obj.pNodes{i} = fusion.trackingArchitecture.internal.TrackingNode.loadobj(s.pNodes{i});
            end
            obj.pNumTrackers = s.pNumTrackers;
            obj.pNumFusers = s.pNumFusers;
            obj.pNodeIndices = s.pNodeIndices;
            obj.pIsFuser = s.pIsFuser;
            obj.pIsTracker = s.pIsTracker;
            obj.pOutput = s.pOutput;
            
            % Set properties related to object setup
            obj.pPrevStepTracks = s.pPrevStepTracks;
            obj.pCurrentStepTracks = s.pCurrentStepTracks;
            obj.pToFuser = s.pToFuser;
            obj.pLocalToFuser = s.pLocalToFuser;
            obj.pNodeLevels = s.pNodeLevels;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Save properties related to object construction
            s.pNodes = cell(1,numel(obj.pNodes));
            s.pNumTrackers = obj.pNumTrackers;
            s.pNumFusers = obj.pNumFusers;
            for i = 1:numel(obj.pNodes)
                s.pNodes{i} = saveobj(obj.pNodes{i});
            end
            s.pNodeIndices = obj.pNodeIndices;
            s.pIsFuser = obj.pIsFuser;
            s.pIsTracker = obj.pIsTracker;
            s.pOutput = obj.pOutput;
            
            % Save properties related to object setup
            s.pPrevStepTracks = obj.pPrevStepTracks;
            s.pCurrentStepTracks = obj.pCurrentStepTracks;
            s.pToFuser = obj.pToFuser;
            s.pLocalToFuser = obj.pLocalToFuser;
            s.pNodeLevels = obj.pNodeLevels;
        end
        
        function varargout = stepImpl(obj,detections,tracks,t)
            
            % Inputs may need to defer validation to when they exist
            
            validateDetections(obj, detections)
            validateTracks(obj, tracks)
            
            % Save previous step tracks for delayed nodes
            obj.pPrevStepTracks = obj.pCurrentStepTracks;
            
            % Schedule nodes ready for update at this time
            toRun = schedule(obj,t);
            
            % Step nodes
            stepNodes(obj, toRun, detections, tracks, t);
            
            % Provide output
            tracks = formatOutput(obj);
            [varargout{1:nargout}] = tracks{:};
        end
    end
    methods(Access={?trackingArchitecture,?matlab.unittest.TestCase})
        function [cycles,delayedEdges,level,orderedNodes] = analyzeTopology(obj)
            %analyzeTopology Analyze the directed graph topology of the architecture
            % analyzeTopology(OBJ) Analyzes the tracking architecture as a directed
            % graph. It looks for algebraic loops (cycles), edges that must be delayed
            % because of that, and puts each node in its level to define which runs
            % first, second, etc.
            
            trackerInds = zeros(1,numel(obj.Trackers));
            trackerInds(:) = cellfun(@(c) c.TrackerIndex, obj.Trackers);
            % Go through the track fusers and collect their sources and targets
            numFusers = numel(obj.TrackFusers);
            if numFusers == 0
                cycles = {};
                delayedEdges = zeros(0,2);
                level = ones(1, numel(obj.Trackers));
                orderedNodes = trackerInds;
                return
            end
            
            s = zeros(0,1);
            t = zeros(0,1);
            for i = 1:numFusers
                thisFuser = obj.TrackFusers{i};
                inds = sourceIndices(thisFuser);
                s(end+1:end+numel(inds)) = inds;
                t(end+1:end+numel(inds)) = thisFuser.FuserIndex;
            end
            g = digraph(s,t);
            cycles = allcycles(g);
            
            % delayedEdges are edges along which tracks from previous time step will be
            % passed
            delayedEdges = zeros(0,2);
            for i = 1:numel(cycles)
                thisCycle = cycles{i};
                for j = 1:numel(thisCycle)-1
                    delayedEdges = addDelayedEdge(delayedEdges, thisCycle(j:j+1));
                end
                delayedEdges = addDelayedEdge(delayedEdges, [thisCycle(end),thisCycle(1)]);
            end
            
            % levels describes, for each Tracker or Fuser index, when to be stepped.
            % Trackers are always stepped first. TrackFusers whose sources are either
            % trackers or a delayedEdge can go next. Every other tracker is one level
            % up from the last one we defined.
            
            % Remove delayed edges to get the initial order.
            g = rmedge(g,delayedEdges(:,1),delayedEdges(:,2));
            orderedNodes = toposort(g,'order','stable');
            level = zeros(1,numel(orderedNodes));
            fuserInds = cellfun(@(c) c.FuserIndex, obj.TrackFusers);
            trackerNodes = any(orderedNodes == trackerInds(:),1);
            level(trackerNodes) = 1;
            trackInputInds = setdiff(orderedNodes, union(trackerInds,fuserInds));
            trackInputNodes = any(orderedNodes == trackInputInds(:),1);
            level(trackInputNodes) = 1;
            
            for ind = 1:numel(orderedNodes)
                for i = 1:ind-1
                    paths = allpaths(g,orderedNodes(i),orderedNodes(ind));
                    pathlens = cellfun(@(c) numel(c), paths);
                    for j = 1:numel(paths)
                        level(ind) = max(level(ind), pathlens(j) + level(i)) - 1;
                    end
                end
            end
            
            % There might be nodes that have not been put into a level.
            % These nodes are only connected through delayed edges
            notInLevel = orderedNodes(level==0);
            for i = 1:numel(notInLevel)
                thisNode = notInLevel(i);
                levelIndex = orderedNodes == notInLevel(i);
                leadingNodes = delayedEdges(delayedEdges(:,2)==thisNode,1);
                tf = ismember(orderedNodes,leadingNodes);
                level(levelIndex) = max(level(tf) + 2) - 1;
            end
            
            % Get rid of fuser inputs that are coming from architecture inputs
            level = level(~trackInputNodes);
            orderedNodes = orderedNodes(~trackInputNodes);
            trackerNodes = trackerNodes(~trackInputNodes);
            
            % Reorganize nodes by first having all tracker nodes and then
            % all fuser nodes
            level = [level(trackerNodes),level(~trackerNodes)];
            orderedNodes = [orderedNodes(trackerNodes),orderedNodes(~trackerNodes)];
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

function delayedEdges = addDelayedEdge(delayedEdges, candidateEdge)
if ~any(all(candidateEdge == delayedEdges,2),1)
    delayedEdges(end+1,:) = candidateEdge;
end
end