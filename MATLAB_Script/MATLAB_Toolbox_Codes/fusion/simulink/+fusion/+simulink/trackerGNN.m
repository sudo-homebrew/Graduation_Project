classdef (StrictDefaults,Hidden) trackerGNN <  trackerGNN & ...
        matlabshared.tracking.internal.fusion.AbstractSimulinkTracker
    %fusion.simulink.trackerGNN Tracking object using GNN assignment in Simulink

    % Copyright 2021 The MathWorks, Inc.

    %#codegen

    properties(Nontunable)
        %TimeInputSource   Prediction time source
        %   Set this property to 'Input port' if you want to pass the
        %   prediction time as an input to step. Set this property
        %   to 'Auto' if you want the prediction time to be inherited from
        %   Simulink. This property only works in Simulink.
        %
        %   Default: 'Input port'
        TimeInputSource = 'Input port'
    end

    properties(Nontunable)
        %StateParametersSimulink  Nontunable StateParameters
        %
        StateParametersSimulink = struct;
    end

    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});
    end

    properties(Nontunable)
        %TentativeTracksOutputPort  Enable tentative tracks output
        %   Set this property to true if you want to get tentative tracks
        %   as an additional output.
        TentativeTracksOutputPort = false

        %AllTracksOutputPort  Enable all tracks output
        %   Set this property to true if you want to get all the tracks
        %   as an additional output.
        AllTracksOutputPort = false

        %InfoOutputPort  Enable info output
        %   Set this property to true if you want to get info
        %   as an additional output.
        InfoOutputPort (1, 1) logical = false
        %HasStateParametersInput Enable state parameters input port.
        %   Set this property to true to update state parameters as an
        %   input with each time step.
        %
        %   Default: false
        HasStateParametersInput (1, 1) logical = false;
    end

    properties(Nontunable)
        %BusNameSource Source of output bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name.
        %
        %   Default: 'Auto'
        BusNameSource = 'Auto';

        %BusName Specify an output bus name
        BusName = char.empty(1,0)

        %BusName2Source Source of output Information bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name.
        %
        %   Default: 'Auto'
        BusName2Source = 'Auto'

        %BusName2 Specify an output Information bus name.
        BusName2 = char.empty(1,0)
    end

    properties(Hidden,Constant)
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
    end

    properties(Constant, Access=protected)
        %   pBusPrefix A string that captures the base output bus name
        %   An output bus name are created by the object. It will have the
        %   name given here, appended by the number of tracks.
        %   Additional sub buses are created as well, e.g., for the tracks.
        pBusPrefix = {'BusTrackerGNN','BusTrackerInfo'}
    end

    properties(Dependent,Access=protected)
        pOutputSelector
        pInputSelector
    end


    %% Methods
    methods
        function val = get.BusName(obj)
            val = obj.BusName;
            val = getBusName(obj,val);
        end

        function set.BusName(obj,val)
            validateBusName(obj,val,'BusName')
            obj.BusName = val;
        end

        function val = get.BusName2(obj)
            val = obj.BusName2;
            val = getBusName(obj,val,2);
        end

        function set.BusName2(obj,val)
            validateBusName(obj,val,'BusName2')
            obj.BusName2 = val;
        end

        function selector = get.pOutputSelector(obj)
            %1st output is confirmed tracks and is always available.
            selector = [true, obj.TentativeTracksOutputPort,obj.AllTracksOutputPort,obj.InfoOutputPort];
        end

        function selector = get.pInputSelector(obj)
            %1st Input is Detections and is always available.
            selector = [true,strcmpi(obj.TimeInputSource, 'Input port'),obj.HasCostMatrixInput,...
                obj.HasDetectableTrackIDsInput,obj.HasStateParametersInput];
        end

        function set.StateParametersSimulink(obj,value)
            setStateParameters(obj,value);
            obj.StateParametersSimulink = value;
        end

    end

    methods(Access = protected)        
        function flag = isInputSizeMutableImpl(obj, index)
            out = inputSizeMutability(obj);
            out = out(obj.pInputSelector);
            flag = out(index);
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = sum(obj.pInputSelector);
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = {'Global';'Nearest Neighbor';'Multi Object Tracker'}; % Use class name
        end      
        
        function setupImpl(obj,detections,varargin)
            % Perform one-time calculations, such as computing constants
            setupImpl@trackerGNN(obj,detections,varargin{:});
            setupSimulinkTracker(obj);
        end

        function [dets,time] = processInputs(obj,detections,varargin)
            if obj.pHasTimeInput
                time = varargin{1};
            else
                time = getCurrentTime(obj);
            end
            setStateParamsFromInputs(obj,varargin{:});
            [dets,time]= processInputs@trackerGNN(obj,detections,time);
        end
        
        function varargout = processOutputs(obj,info)
            [varargout{1:nargout}] = formatSimulinkOutputs(obj,info);
        end
       
        function flag = isInfoRequested(obj,~)
            flag =  obj.InfoOutputPort;
        end

        function setMaxCostSize(obj)
            obj.cCostCalculator.MaxCostSize = [obj.MaxNumTracks, getMaxNumDetectionsPerSensor(obj)];
        end

        function setAssignmentResults(~,~,~,~)
            %Nothing to do in Simulink.
        end 

        function setupDetectionManager(obj,detections)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker
            setupDetectionManager@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj,detections);
        end

        function setupAssigner(obj)
            %setup assigner and set pIsInSimulink to true.
            setupAssigner@trackerGNN(obj);
            obj.cAssigner.pIsInSimulink = true;
        end

        function track = allocateTrack(obj,sampleDetection)
            track = allocateTrack@trackerGNN(obj,sampleDetection);
            track.pIsInSimulink = true;
        end

        function setupOOSMHandler(obj)
            %Setup OOSM handlar and set IsInSimulink to true.
            setupOOSMHandler@trackerGNN(obj);
            if ~isempty( obj.cOOSMHandler)
                obj.cOOSMHandler.IsInSimulink = true;
            end
        end
        function out = getMaxNumInputDetections(obj)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker
            out = getMaxNumInputDetections@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj);
        end

        function resetLastTimeStamp(obj)
            if ~obj.pHasTimeInput
                sts = getSampleTime(obj);
                obj.pLastTimeStamp = cast(-sts.SampleTime, 'like', obj.pLastTimeStamp);
            else
                resetLastTimeStamp@trackerGNN(obj);
            end
        end

        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            obj.pHasTimeInput = strcmpi(obj.TimeInputSource, 'Input port');
            validatePropertiesImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            validateTrackerProperties(obj);
        end      

        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog

            flag = isInactivePropertyImpl@trackerGNN(obj, prop);
            flag = flag || isPropInactiveInSimulink(obj,prop);
        end

        function validateInputsImpl(obj,~,varargin)
            % Validate inputs to the step method at initialization
            if obj.pHasTimeInput
                validateTimeInput(obj,varargin{:});                
            end
            if obj.HasDetectableTrackIDsInput
                validateDetectableTrackIDs(obj,varargin{:});
            end
        end

        function detectables = getDetectableTrackIDsFromInput(obj,varargin)
            %Get detectable track ids from input in Simulink.
            detectables = getDetectableTrackIDsInSimulink(obj,varargin{:});
        end

        function costMatrix = getCostMatrixFromInput(obj,varargin)
            %Get costMatrix from input in Simulink.
            costMatrix = getCostMatrixinSimulink(obj,varargin{:}) ;
        end

        function varargout = getOutputDataTypeImpl(obj)
            out = outputDataTypes(obj,obj.InfoOutputPort);
            varargout = out(obj.pOutputSelector);
        end

        function varargout= getInputNamesImpl(obj)
            % Return input port names for System block
            names = inputNames(obj);
            varargout = names(obj.pInputSelector);
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            names = outputNames(obj);
            varargout = names(obj.pOutputSelector);
        end

        function varargout = getOutputSizeImpl(~)
           [varargout{1:nargout}] = deal([1,1]);
        end

        function varargout= isOutputComplexImpl(~)
            [varargout{1:nargout}] = deal(false);
        end

        function varargout = isOutputFixedSizeImpl(~)
            [varargout{1:nargout}] = deal(true);
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system
            num = sum(obj.pOutputSelector);
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            % Save the base class information
            s = saveObjectImpl@trackerGNN(obj);
            s = saveSimulinkProps(obj,s);
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            s = loadSimulinkProps(obj,s,wasLocked);
            loadObjectImpl@trackerGNN(obj,s,wasLocked);
        end
    end

    methods(Hidden)
        function info = padInfoData(obj,infoTrack)
            % Variable size data can not be logged in simulink hence
            % padding the data to make it fixed size.
            
            oosmDetInds = infoTrack.OOSMDetectionIndices; % Already fixed-size
            
            num = obj.MaxNumTracks;
            trIDs = zeros(1,num,'like',infoTrack.TrackIDsAtStepBeginning);
            if ~isempty(infoTrack.TrackIDsAtStepBeginning)
                numTrsInitial = numel(infoTrack.TrackIDsAtStepBeginning);
                trIDs(1:numTrsInitial) = infoTrack.TrackIDsAtStepBeginning;
            end
           
            costMat = zeros(num,maxNumDetections(obj.cDetectionManager),obj.pClassToUse);
            if ~isempty(infoTrack.CostMatrix)
                numTracks = size(infoTrack.CostMatrix,1);
                numDets   = size(infoTrack.CostMatrix,2);
                costMat(1:numTracks,1:numDets)= infoTrack.CostMatrix(1:numTracks,1:numDets);
            end
            
            assgn = zeros(num,2,'like',infoTrack.Assignments);
            if ~isempty(infoTrack.Assignments)
                numAssignments = size(infoTrack.Assignments,1);
                assgn(1:numAssignments,:) = infoTrack.Assignments;
            end
            
            unasTrs = zeros(1,num,'like',infoTrack.UnassignedTracks);
            if ~isempty(infoTrack.UnassignedTracks)
                numUnassignTrs = numel(infoTrack.UnassignedTracks);
                unasTrs(1:numUnassignTrs) = infoTrack.UnassignedTracks;
            end
            
            unassgnDets = zeros(num,1,'like',infoTrack.UnassignedDetections);
            if ~isempty(infoTrack.UnassignedDetections)
                numUnassignedDets = numel(infoTrack.UnassignedDetections);
                unassgnDets(1:numUnassignedDets) = infoTrack.UnassignedDetections;
            end
            
            initTrIDs = zeros(1,num,'like',infoTrack.InitiatedTrackIDs);
            if ~isempty(infoTrack.InitiatedTrackIDs)
                numInitiatedIDs = numel(infoTrack.InitiatedTrackIDs);
                initTrIDs(1:numInitiatedIDs) = infoTrack.InitiatedTrackIDs;
            end
            
            delTrIDs = zeros(1,num,'like',infoTrack.DeletedTrackIDs);
            if ~isempty(infoTrack.DeletedTrackIDs)
                numDelTrsIDs = numel(infoTrack.DeletedTrackIDs);
                delTrIDs(1:numDelTrsIDs) = infoTrack.DeletedTrackIDs;
            end
            
            trsIDEnd = zeros(1,num,'like',infoTrack.TrackIDsAtStepEnd);
            if ~isempty(infoTrack.TrackIDsAtStepEnd)
                numTrsStepEnd = numel(infoTrack.TrackIDsAtStepEnd);
                trsIDEnd(1:numTrsStepEnd) = infoTrack.TrackIDsAtStepEnd;
            end
            
            info = struct(...
                'OOSMDetectionIndices',oosmDetInds,...
                'TrackIDsAtStepBeginning',trIDs, ...
                'CostMatrix', costMat, ...
                'Assignments', assgn, ...
                'UnassignedTracks', unasTrs, ...
                'UnassignedDetections', unassgnDets, ...
                'InitiatedTrackIDs', initTrIDs, ...
                'DeletedTrackIDs', delTrIDs, ...
                'TrackIDsAtStepEnd', trsIDEnd ...
                );
            
            if isfield(infoTrack,'OOSMHandling') && ~isempty(infoTrack.OOSMHandling)
                info.OOSMHandling = infoTrack.OOSMHandling; % Already padded
            end
            if isfield(infoTrack,'MaxNumDetectionsPerCluster')
                info.MaxNumDetectionsPerCluster = infoTrack.MaxNumDetectionsPerCluster;
            end
            if isfield(infoTrack,'MaxNumTracksPerCluster')
                info.MaxNumTracksPerCluster = infoTrack.MaxNumTracksPerCluster;
            end
        end

        function infoStruct = defaultInfo(obj,maxDets,classToUse)
            num = obj.MaxNumTracks;
            if nargin == 1
                maxDets = maxNumDetections(obj.cDetectionManager);
                classToUse = obj.pClassToUse;
            end
            infoStruct = struct(...
                'OOSMDetectionIndices', zeros(1,maxDets,'uint32'), ...
                'TrackIDsAtStepBeginning', zeros(1,num,'uint32'), ...
                'CostMatrix', zeros(num,maxDets,classToUse), ...
                'Assignments', zeros(num,2,'uint32'), ...
                'UnassignedTracks', zeros(1,num,'uint32'), ...
                'UnassignedDetections', zeros(num,1,'uint32'), ...
                'InitiatedTrackIDs', zeros(1,num,'uint32'), ...
                'DeletedTrackIDs', zeros(1,num,'uint32'), ...
                'TrackIDsAtStepEnd', zeros(1,num,'uint32'));
            if strcmpi(obj.OOSMHandling, 'Retrodiction')
                infoStruct.OOSMHandling = struct( ...
                    'DiscardedDetections', zeros(1,maxDets,'uint32'), ...
                    'CostMatrix', zeros(num,maxDets,classToUse), ...
                    'Assignments', zeros(num,2,'uint32'), ...
                    'UnassignedDetections', zeros(maxDets,1,'uint32'));
            end
            if strcmpi(obj.AssignmentClustering,'on')
                infoStruct.MaxNumDetectionsPerCluster = uint32(0);
                infoStruct.MaxNumTracksperCluster = uint32(0);
            end
        end
        %------------------------------------------------------------------
    end

    methods(Static)
        function busName = createBus(varargin)
            % gnnobj.createBus Create Simulink output bus
            %   The track structs generated by the trackerGNN MATLAB
            %   System object are output on a bus in Simulink. This function
            %   creates the Simulink bus object in the MATLAB base workspace.
            %   This bus object defines the datatype for the tracks output
            %   ports.
            %
            %   busName = gnnobj.createBus(blkPath) creates a
            %   bus from the MATLAB System block specified by the full block
            %   path, blkPath. The bus is created in the MATLAB base workspace
            %   and the name of the created bus is returned as busName. The
            %   block's "Source of output bus name" parameter must be set to
            %   'Property'. The name of the bus is specified on the block's
            %   dialog.
            %
            %   busName = gnnobj.createBus(blkPath, busName)
            %   creates a bus named busName from the MATLAB System block,
            %   blkPath.

            busName = createBus@matlabshared.tracking.internal.SimulinkBusPropagation(varargin{:});
        end
    end
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:gnnTrackerTitle', ...
                'Text',	 'fusion:block:gnnTrackerDesc');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            absTrkrSection = matlabshared.tracking.internal.fusion.AbstractSimulinkTracker.getBusPropertyGroups;

            propNamesTracker = {'TrackerIndex', 'FilterInitializationFcn','MaxNumTracks', ...
                'MaxNumSensors','OOSMHandling','MaxNumOOSMSteps',...
                'StateParametersSimulink','HasStateParametersInput','EnableMemoryManagement'};
            trackerSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerGNN','',propNamesTracker, {9,'internal:TrackerMemoryManagementUtilities'});
            
            propNamesAssignment = {'Assignment','CustomAssignmentFcn', ...
                'AssignmentThreshold','AssignmentClustering'};
            assignmentSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerGNN','Assignment',propNamesAssignment);

            trackerAndAssignmentGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerGNN:GroupTrackerSection')), ...
                'Sections', [trackerSection assignmentSection]);

            propNamesLogic = {'TrackLogic', 'pHistoryConfThreshold','pScoreConfThreshold', ...
                'pHistoryDelThreshold','pScoreDelThreshold','DetectionProbability',...
                'FalseAlarmRate', 'Volume', 'Beta'};
            logicSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerGNN','',propNamesLogic);
            logicGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerGNN:GrouplogicSection')), ...
                'Sections', logicSection);
            
            busList = absTrkrSection.PropertyList;
            redirectList = {1 , 'SimulinkBusPropagation'; 2, 'SimulinkBusPropagation'};
            portsList = {busList{:},'BusName2Source','BusName2'};
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerGNN','OutputPortSettings', portsList, redirectList);
            
            propIOList = {'TimeInputSource', 'HasCostMatrixInput', 'HasDetectableTrackIDsInput', ...
                'TentativeTracksOutputPort','AllTracksOutputPort','InfoOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerGNN','InputsOutputs', propIOList);
            ioGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerGNN:GroupIO')), ...
                'Sections', [ioSection,portsSection]);
            
            memManagementGroup = fusion.internal.TrackerMemoryManagementUtilities.getPropertyGroupsImpl();

            groups = [trackerAndAssignmentGroup, logicGroup, ioGroup memManagementGroup];
        end
    end

    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end
end
