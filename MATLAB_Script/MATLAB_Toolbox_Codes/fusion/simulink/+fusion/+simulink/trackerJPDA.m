classdef (StrictDefaults,Hidden) trackerJPDA <  trackerJPDA & ...
        matlabshared.tracking.internal.fusion.AbstractSimulinkTracker

    % Copyright 2021 The MathWorks, Inc.

    %#codegen

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
        %TimeInputSource   Prediction time source
        %   Set this property to 'Input port' if you want to pass the
        %   prediction time as an input to step. Set this property
        %   to 'Auto' if you want the prediction time to be inherited from
        %   Simulink. This property only works in Simulink.
        %
        %   Default: 'Input port'
        TimeInputSource = 'Input port'
    end

    properties (Nontunable)
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
        %   use the default bus name in Simulink. This property only works
        %   in Simulink.
        %
        %   Default: 'Auto'
        BusName2Source = 'Auto'

        %BusName2 Specify an output Information bus name
        BusName2 = char.empty(1,0)
    end
    properties(Constant, Access=protected)
        %pBusPrefix A string that captures the base output bus name
        %   An output bus name are created by the object. It will have the
        %   name given here, appended by the number of tracks.
        %   Additional sub buses are created as well, e.g., for the tracks.
        pBusPrefix = {'BusTrackerJPDA', 'BusTrackerInfo'}
    end

    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});        
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
    end

    properties(Nontunable)
        %StateParametersSimulink  Nontunable StateParameters
        %
        StateParametersSimulink = struct;
    end

    properties(Dependent,Access=protected)
        pOutputSelector
        pInputSelector
    end

    properties(Access = protected, Nontunable)
        %pMaxNumDetections A scalar value that defines the maximum number
        %of detections in Simulink. This value gets set depending upon the
        %size of the detection input bus.
        pMaxNumDetections = 100
    end

    methods
        function obj = trackerJPDA(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end

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
        %------------------------------------------------------------------
        function set.BusName2(obj,val)
            validateBusName(obj,val,'BusName2')
            obj.BusName2 = val;
        end
        function selector = get.pOutputSelector(obj)
            %1st output is confirmed tracks and is always available.
            selector = [true, obj.TentativeTracksOutputPort,...
                obj.AllTracksOutputPort,obj.InfoOutputPort];
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
        function setupImpl(obj, detections, varargin)           
            setupImpl@trackerJPDA(obj,detections,varargin{:});
            setupSimulinkTracker(obj);
            obj.pMaxNumDetections =  obj.cDetectionManager.maxNumDetections;
        end

        function [dets,time] = processInputs(obj,detections,varargin)
            if obj.pHasTimeInput
                time = varargin{1};
            else
                time = getCurrentTime(obj);
            end
            setStateParamsFromInputs(obj,varargin{:});
            [dets,time] = processInputs@trackerJPDA(obj,detections,time);
        end
        
        function varargout = processOutputs(obj,info)
            [varargout{1:nargout}] = formatSimulinkOutputs(obj,info);
        end

        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog

            flag = isInactivePropertyImpl@trackerJPDA(obj, prop);
            flag = flag || isPropInactiveInSimulink(obj,prop);
        end
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            obj.pHasTimeInput = strcmpi(obj.TimeInputSource, 'Input port');
            validatePropertiesImpl@trackerJPDA(obj);
        end

        function flag = isInfoRequested(obj,~)
            flag =  obj.InfoOutputPort ;
        end

        function detectables = getDetectableTrackIDsFromInput(obj,varargin)
            %Get detectable track ids from input in Simulink.
            detectables = getDetectableTrackIDsInSimulink(obj,varargin{:});
        end

        function costMatrix = getCostMatrixFromInput(obj,varargin)
            %Get costMatrix from input in Simulink.
            costMatrix = getCostMatrixinSimulink(obj,varargin{:});
        end

        function validateInputsImpl(obj,~,varargin)
            % Validate time input
            if obj.pHasTimeInput
                validateTimeInput(obj,varargin{:});
            end
            if obj.HasDetectableTrackIDsInput
                validateDetectableTrackIDs(obj,varargin{:});
            end
        end

        function setAssignmentResults(~,~,~,~)
            %Nothing to do in Simulink.
        end
        function setupDetectionManager(obj,detections)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker
            setupDetectionManager@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj,detections);
        end
        function track = allocateTrack(obj,sampleDetection)
            track = allocateTrack@trackerJPDA(obj,sampleDetection);
            track.pIsInSimulink = true;
        end
        function out = getMaxNumInputDetections(obj)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker
            out = getMaxNumInputDetections@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj);
        end

        function releaseImpl(obj)
            % Release resources, such as file handles
            releaseImpl@trackerJPDA(obj);
            obj.pLastTrackID = uint32(0);
            if ~obj.pHasTimeInput
                sts = getSampleTime(obj);
                obj.pLastTimeStamp = cast(-sts.SampleTime, 'like', obj.pLastTimeStamp);
            end
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@trackerJPDA(obj);            
            s = saveSimulinkProps(obj,s);
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s         
            if isfield(s,'pMaxNumDetections')
                obj.pMaxNumDetections = s.pMaxNumDetections;
            end
            s = loadSimulinkProps(obj,s,wasLocked);
            loadObjectImpl@trackerJPDA(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the trackerJPDA
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy public properties
            newTracker = cloneImpl@trackerJPDA(obj);
            newTracker.pHasTimeInput = obj.pHasTimeInput;
            if coder.internal.is_defined(obj.cDetectionManager) %Only happens after setup
                % Next, copy the rest of the private properties
                newTracker.pMaxNumDetections = obj.pMaxNumDetections;   
            end
        end

        %% Simulink methods
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = {'Joint Probabilistic';'Data Association';'Multi Object Tracker'};
        end
        
        function flag = isInputSizeMutableImpl(obj, index)
            out = inputSizeMutability(obj);
            out = out(obj.pInputSelector);
            flag = out(index);
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = sum(obj.pInputSelector);
        end
        
        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            names = inputNames(obj);
            varargout = names(obj.pInputSelector);
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            names = outputNames(obj);
            varargout = names(obj.pOutputSelector);
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system
            num = sum(obj.pOutputSelector);
        end

        function varargout = getOutputSizeImpl(~)
            [varargout{1:nargout}] = deal([1,1]);
        end

        function  varargout = getOutputDataTypeImpl(obj)
            out = outputDataTypes(obj,obj.InfoOutputPort);
            varargout = out(obj.pOutputSelector);
        end

        function varargout = isOutputComplexImpl(~)
            [varargout{1:nargout}] = deal(false);
        end

        function varargout= isOutputFixedSizeImpl(~)
            [varargout{1:nargout}] = deal(true);
        end
    end

    methods(Hidden)
        function infoStruct = defaultInfo(obj, maxDets, classToUse)
            num = obj.MaxNumTracks;
            if nargin == 1
                maxDets = obj.pMaxNumDetections;
                classToUse = obj.pClassToUse;
            end
            clustersInfo = struct('DetectionIndices',uint32(zeros(1,num)),...
                'TrackIDs', uint32(zeros(1,num)),...
                'ValidationMatrix', false(maxDets,num+1),...
                'SensorIndex',uint32(zeros(1)),...
                'TimeStamp' ,zeros(1,classToUse),...
                'MarginalProbabilities',zeros(maxDets+1,num,classToUse));
            maxNum = max(num,maxDets);
            infoStruct = struct(...
                'OOSMDetectionIndices', uint32(zeros(1,maxDets)), ...
                'TrackIDsAtStepBeginning', uint32(zeros(1,num)), ...
                'UnassignedTracks', uint32(zeros(1,num)), ...
                'UnassignedDetections', uint32(zeros(1,num)), ...
                'CostMatrix',zeros(num,maxDets,classToUse), ...
                'Clusters', repmat(clustersInfo,1,maxNum), ...
                'InitializedTrackIDs', uint32(zeros(1,num)), ...
                'DeletedTrackIDs',uint32(zeros(1,num)), ...
                'TrackIDsAtStepEnd', uint32(zeros(1,num)));
            
            if obj.EnableMemoryManagement
                infoStruct.MaxNumDetectionsPerCluster = uint32(0);
                infoStruct.MaxNumTracksPerCluster = uint32(0);
            end
        end

        function info = padInfoData(obj,infoTrack)
            % Variable size data can not be logged in simulink hence
            % padding the data to make it fixed size.

            oosmDetInds = infoTrack.OOSMDetectionIndices; % Already fixed-size

            num = obj.MaxNumTracks;
            trIDs = zeros(1,num,'like',infoTrack.TrackIDsAtStepBeginning);
            if ~isempty(infoTrack.TrackIDsAtStepBeginning)
                numTrsInitial = numel(infoTrack.TrackIDsAtStepBeginning);
                trIDs(1:numTrsInitial)= infoTrack.TrackIDsAtStepBeginning;
            end

            costMat = zeros(num,obj.pMaxNumDetections,obj.pClassToUse);
            if ~isempty(infoTrack.CostMatrix)
                numTracks = size(infoTrack.CostMatrix,1);
                numDets   = size(infoTrack.CostMatrix,2);
                costMat(1:numTracks,1:numDets)= infoTrack.CostMatrix(1:numTracks,1:numDets);
            end

            unasTrs = zeros(1,num,'like',infoTrack.UnassignedTracks);
            if ~isempty(infoTrack.UnassignedTracks)
                numUnassignedTrs = numel(infoTrack.UnassignedTracks);
                unasTrs(1:numUnassignedTrs) = infoTrack.UnassignedTracks;
            end

            unassgnDets = zeros(1,num,'like',infoTrack.UnassignedDetections);
            if ~isempty(infoTrack.UnassignedDetections)
                numUnassignedDets = numel(infoTrack.UnassignedDetections);
                unassgnDets(1:numUnassignedDets) = infoTrack.UnassignedDetections;
            end

            initTrIDs = zeros(1,num,'like',infoTrack.InitializedTrackIDs);
            if ~isempty(infoTrack.InitializedTrackIDs)
                numInitiatedIDs = numel(infoTrack.InitializedTrackIDs);
                initTrIDs(1:numInitiatedIDs)=infoTrack.InitializedTrackIDs;
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
            infoStruct = defaultInfo(obj);
            clustersInfo = infoStruct.Clusters;

            if ~isempty(infoTrack.Clusters)
                for i = 1:numel(infoTrack.Clusters{1})
                    structInfo = infoTrack.Clusters{1};
                    detID = zeros(1,num,'like',structInfo(i).DetectionIndices);
                    if ~isempty(structInfo(i).DetectionIndices)
                        numDetID = numel(structInfo(i).DetectionIndices);
                        detID(1,1:numDetID)= structInfo(i).DetectionIndices;
                    end

                    trsID = zeros(1,num,'like',structInfo(i).TrackIDs);
                    if ~isempty(structInfo(i).TrackIDs)
                        numTrsID = numel(structInfo(i).TrackIDs);
                        trsID(1:numTrsID) = structInfo(i).TrackIDs;
                    end

                    valMat = zeros(obj.pMaxNumDetections,num+1,'like',infoTrack.Clusters{i}.ValidationMatrix);
                    if ~isempty(structInfo(i).ValidationMatrix)
                        d1 = size(structInfo(i).ValidationMatrix,1);
                        d2 = size(structInfo(i).ValidationMatrix,2);
                        valMat(1:d1,1:d2)= structInfo(i).ValidationMatrix;
                    end

                    if ~isempty(structInfo(i).SensorIndex)
                        sID = zeros(1,'like',structInfo(i).SensorIndex);
                        numSID = numel(structInfo(i).SensorIndex);
                        sID(1:numSID) = structInfo(i).SensorIndex;
                    end

                    time = zeros(1,'like',structInfo(i).TimeStamp);
                    if ~isempty(structInfo(i).TimeStamp)
                        numTime = numel(structInfo(i).TimeStamp);
                        time(1:numTime) = structInfo(i).TimeStamp;
                    end

                    margProbs = zeros(obj.pMaxNumDetections+1,num,'like',structInfo(i).MarginalProbabilities);
                    if ~isempty(structInfo(i).MarginalProbabilities)
                        dim1 = size(structInfo(i).MarginalProbabilities,1);
                        dim2 = size(structInfo(i).MarginalProbabilities,2);
                        margProbs(1:dim1,1:dim2)= structInfo(i).MarginalProbabilities;
                    end

                    clustersInfo(i).DetectionIndices = detID;
                    clustersInfo(i).TrackIDs = trsID;
                    clustersInfo(i).ValidationMatrix = valMat;
                    clustersInfo(i).SensorIndex = sID;
                    clustersInfo(i).TimeStamp = time;
                    clustersInfo(i).MarginalProbabilities = margProbs;
                end
            end
            info = struct(...
                'OOSMDetectionIndices',oosmDetInds,...
                'TrackIDsAtStepBeginning',trIDs, ...
                'UnassignedTracks',unasTrs, ...
                'UnassignedDetections',unassgnDets, ...
                'CostMatrix',costMat, ...
                'Clusters',clustersInfo, ...
                'InitializedTrackIDs',initTrIDs, ...
                'DeletedTrackIDs',delTrIDs, ...
                'TrackIDsAtStepEnd',trsIDEnd);
            if isfield(infoTrack,'MaxNumDetectionsPerCluster')
                info.MaxNumDetectionsPerCluster = infoTrack.MaxNumDetectionsPerCluster;
            end
            if isfield(infoTrack,'MaxNumTracksPerCluster')
                info.MaxNumTracksPerCluster = infoTrack.MaxNumTracksPerCluster;
            end
        end
    end

   methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:jpdaTrackerTitle', ...
                'Text',	 'fusion:block:jpdaTrackerDesc');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            
            absTrkrSection = matlabshared.tracking.internal.fusion.AbstractSimulinkTracker.getBusPropertyGroups;
            
            propNamesTracker = {'TrackerIndex', 'FilterInitializationFcn', ...
                'MaxNumEvents','EventGenerationFcn', 'MaxNumTracks','MaxNumSensors', ...
                'TimeTolerance', 'OOSMHandling','MaxNumOOSMSteps', 'StateParametersSimulink', 'HasStateParametersInput','EnableMemoryManagement'};
            trackerSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerJPDA','',propNamesTracker, {12,'internal:TrackerMemoryManagementUtilities'});
            trackerGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerJPDA:GroupTrackerSection')), ...
                'Sections', trackerSection);
            
            propNamesAssignment = {'AssignmentThreshold','InitializationThreshold', ...
                'DetectionProbability', 'ClutterDensity'};
            asssignmentSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerJPDA','',propNamesAssignment);
            assignmentGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerJPDA:GroupAssignmentSection')), ...
                'Sections',asssignmentSection);
            
            propNamesLogic = {'TrackLogic', 'pHistoryConfThreshold', 'pHistoryDelThreshold',...
                'pIntegratedConfThreshold', 'pIntegratedDelThreshold','HitMissThreshold', ...
                'NewTargetDensity', 'DeathRate'};
            logicSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerJPDA','',propNamesLogic);
            logicGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerJPDA:GrouplogicSection')), ...
                'Sections', logicSection);
            
            busList = absTrkrSection.PropertyList;
            redirectList = {1 , 'SimulinkBusPropagation'; 2, 'SimulinkBusPropagation'};
            portsList = {busList{:},'BusName2Source','BusName2'};
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerJPDA','OutputPortSettings', portsList, redirectList);
            
            propIOList = {'TimeInputSource', 'HasCostMatrixInput', 'HasDetectableTrackIDsInput', ...
                'TentativeTracksOutputPort','AllTracksOutputPort','InfoOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'trackerJPDA','InputsOutputs', propIOList);
            ioGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:trackerJPDA:GroupIO')), ...
                'Sections', [ioSection,portsSection]);
            
            memManagementGroup = fusion.internal.TrackerMemoryManagementUtilities.getPropertyGroupsImpl();
            groups = [trackerGroup, assignmentGroup, logicGroup, ioGroup memManagementGroup];
        end

    end
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end

end
