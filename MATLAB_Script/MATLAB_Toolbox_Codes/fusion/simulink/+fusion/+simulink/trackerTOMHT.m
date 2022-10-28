classdef (StrictDefaults, Hidden) trackerTOMHT < trackerTOMHT & ...
        matlabshared.tracking.internal.fusion.AbstractSimulinkTracker

    % This is the Simulink implementation of trackerTOMHT.
    %
    %   See also: trackerTOMHT
    
    % Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen 
    properties(Nontunable)
        %TentativeTracksOutputPort  Enable tentative tracks output
        %   Set this property to true if you want to get tentative tracks
        %   as an additional output
        TentativeTracksOutputPort = false
        
        %AllTracksOutputPort  Enable all tracks output
        %   Set this property to true if you want to get all the tracks
        %   as an additional output
        AllTracksOutputPort = false
        
        %InfoOutputPort  Enable info output
        %   Set this property to true if you want to get info
        %   as an additional output
        InfoOutputPort (1, 1) logical = false
        
        %AllBranchesOutputPort  Enable track Branches output
        %   Set this property to true if you want to get the current list
        %   of track branches as an additional output
        AllBranchesOutputPort (1, 1) logical = false
        
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

    properties(Nontunable)
        %StateParametersSimulink  Nontunable StateParameters
        %
        StateParametersSimulink = struct;
    end
    
    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
    end

    properties(Constant, Access=protected)
        %pBusPrefix A string that captures the base output bus name
        %   An output bus name are created by the object. It will have the
        %   name given here, appended by the number of tracks.
        %   Additional sub buses are created as well, e.g., for the tracks.
        pBusPrefix = {'BusTrackerTOMHT' , 'BusTrackerInfo'}
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
        %   use the default bus name in Simulink. This property only works
        %   in Simulink.
        %
        %   Default: 'Auto'
        BusName2Source = 'Auto'
        
        %BusName2 Specify an output Information bus name
        BusName2 = char.empty(1,0)
    end
    
    properties(Constant, Access=protected)
        %pConstantTimeTol  A tolerance value for time interval check
        %   Defines a time tolerance that is used when comparing the
        %   detection timestamps with the time interval expected by the
        %   obj. Only applies if TimeInputSource is 'Input Port'
        %
        %   NOTES: the value of the tolerance should be:
        %     1. Large enough to avoid issues in floating point comparison
        %        (e.g., 1.00000001 > 1).
        %     2. Small enough so that no sensor detection in the next
        %        time frame can have a timestamp less than this tolerance
        %        value after the end of the current time frame.
        pConstantTimetol = 1e-5
    end
    
    properties( Access = {?matlabshared.tracking.internal.fusion.TrackManager}, Nontunable)
        %TrackLogic  Type of track confirmation and deletion logic
        TrackLogic = 'Score'
    end
    
    %Public
    methods
        %% Common functions
        function obj = trackerTOMHT(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end

        function set.StateParametersSimulink(obj,value)
            setStateParameters(obj,value);
            obj.StateParametersSimulink = value;
        end
        
        % Bus name validation for buses
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
    end
    %% Protected
    methods(Access=protected)
        
        function setupImpl(obj, detections, varargin)
            % Setup properties that will not be modified later on
            % Perform one-time calculations, such as computing constants
            
            % Setup the core tracker.
            if obj.pHasTimeInput
                setupImpl@trackerTOMHT(obj, detections, varargin{:});
            else
                setupImpl@trackerTOMHT(obj, detections, 0, varargin{:});
            end

            setupSimulinkTracker(obj);            
            
            % Set the AllBrachesPort if detectableBranchIds or Cost matrix
            % is an input to the block.
            if (obj.HasDetectableBranchIDsInput || obj.HasCostMatrixInput)
                obj.AllBranchesOutputPort = true;
            end
        end
        
        function resetImpl(obj)
            % Returns the tracker to its initial state
            resetImpl@trackerTOMHT(obj);
            sts = getSampleTime(obj);
            if sts.SampleTime > 0
                obj.pLastTimeStamp = cast(-sts.SampleTime, 'like', obj.pLastTimeStamp);
            else
                obj.pLastTimeStamp = cast(-eps, 'like', obj.pLastTimeStamp); %Has to be negative to run tracker from t=0
            end
        end
        
        function releaseImpl(obj)
            % Release resources
            releaseImpl@trackerTOMHT(obj);
        end
        %% Save / Load / Clone Impl
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@trackerTOMHT(obj);           
            s = saveSimulinkProps(obj,s);
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            if wasLocked
                if isfield(s, 'pMaxNumDetections')
                    s = rmfield(s, 'pMaxNumDetections');
                end
            end
            s = loadSimulinkProps(obj,s,wasLocked);
            loadObjectImpl@trackerTOMHT(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the simulink trackerTOMHT
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy public properties
            newTracker = cloneImpl@trackerTOMHT(obj);
            
            if coder.internal.is_defined(obj.cDetectionManager) %Only happens after setup
                newTracker.pHasTimeInput = obj.pHasTimeInput;
            end
        end
        
        function varargout = stepImpl(obj, detections, varargin)
            %Process inputs
            time = processInputs(obj, detections, varargin{:});
            
            %Run core algorithm
            if obj.InfoOutputPort
                [hypothesis, cluster, probability, keptScores, info] ...
                    = coreAlgorithm(obj,  varargin{:});
            else
                [hypothesis, cluster, probability, keptScores] ...
                    = coreAlgorithm(obj,  varargin{:});
                info = {};
            end
            
            % Process Output
            [varargout{1:nargout}] = getTrackerOutputs(obj, time, ...
                hypothesis, cluster, probability, keptScores, info);
        end
        
        function [Hypothesis, Cluster, Probability, KeptScores, Information]...
                = coreAlgorithm(obj, varargin)
            [Hypothesis, Cluster, Probability, KeptScores, Information] ...
                = coreAlgorithm@trackerTOMHT(obj, varargin{:});
        end
        
        function time = processInputs(obj, detections, varargin)
            % Extract the detections from detection bus
            
            % Work with and without time input
            if  obj.pHasTimeInput
                % In MATLAB: time is always the 1st part of varargin
                time = varargin{1};
                % Error out if the time input is not greater than obj.
                coder.internal.errorIf(time <= obj.pLastTimeStamp, ...
                    'fusion:simulink:trackerTOMHT:TimeMustIncrease','step');
            else
                time = getCurrentTime(obj);
            end
            % Set the state parameters in the block.
            if obj.HasStateParametersInput
                stParam = varargin{(obj.pHasTimeInput + obj.HasStateParametersInput +...
                    obj.HasCostMatrixInput + obj.HasDetectableBranchIDsInput)};
                setStateParameters(obj,stParam);
            end
            % Pre-process detections (format, time, sensor index)
            step(obj.cDetectionManager, detections, obj.pLastTimeStamp, time);
        end
        
        function predictTracks(obj,time)
            predictTracks@trackerTOMHT(obj,time)
        end
        
        function [conf,tent,all] = organizeTrackOutputs(obj,trs)
            [conf,tent,all] = organizeTrackOutputs@trackerTOMHT(obj,trs);
        end
        
        function varargout = getTrackerOutputs(obj, varargin)
            time  = varargin{1};
            hyps1 = varargin{2};
            clu1  = varargin{3};
            prob1 = varargin{4};
            keptScores = varargin{5};
            info =  varargin{6};
            
            % Predict tracks to update time
            predictTracks(obj,time);
            
            % Update tracker timestamp
            obj.pLastTimeStamp = cast(time,'like',obj.pLastTimeStamp);
            
            % Output tracks
            outputList = true(obj.pNumLiveTracks,1);
            tracks = getTracks(obj, outputList);
            trs = tracks.Tracks(1:tracks.NumTracks);
            
            % Reorder the hypotheses by score:
            if coder.target('MATLAB')
                ts = [trs.TrackLogicState]; % Returns a vector
                trackScores = ts(1:2:end-1)';
            else
                trackScores = zeros(obj.pNumLiveTracks,1,'like',keptScores);
                for ii = 1:obj.pNumLiveTracks
                    trackScores(ii) = trs(ii).TrackLogicState(1);
                end
            end
            hypScores = trackScores(:,1)' * hyps1;
            [~,I] = sort(hypScores,'descend');
            
            outputHyps = hyps1(:,I);
            % Remove clusters if the branches in them are pruned
            if size(clu1,2) > 0
                outputClusters = clu1(:,sum(clu1)>0);
            else % Nothing to remove, already zero columns (clusters)
                outputClusters = clu1;
            end
            outputProbability = prob1;
            
            obj.pHyps = outputHyps;
            obj.pClusters = outputClusters;
            obj.pProbs = outputProbability;
            
            [conf,tent,all] = organizeTrackOutputs(obj,trs);
            
            vargs = cell(1, nargout);
            
            % First output will always be confirmed tracks
            vargs{1} = getTrack(obj, conf);
            
            % Add each optional output based on their flag and order
            if obj.TentativeTracksOutputPort
                vargs{1+obj.TentativeTracksOutputPort} = getTrack(obj, tent);
            end
            
            if obj.AllTracksOutputPort
                vargs{1+obj.TentativeTracksOutputPort+obj.AllTracksOutputPort} = getTrack(obj, all);
            end
            
            if obj.InfoOutputPort
                vargs{1+obj.TentativeTracksOutputPort+obj.AllTracksOutputPort+obj.InfoOutputPort} = getInfo(obj, info);
            end
            
            if obj.AllBranchesOutputPort
                vargs{nargout} = getBranches(obj);
            end
            varargout = vargs;
        end
        
        function tracks =  getTracks(obj,outputList)
           tracks = getTracksSimulink(obj,obj.pTracksList,outputList,obj.pMaxNumBranches);
        end
        
        function validatePropertiesImpl(obj)
            validatePropertiesImpl@trackerTOMHT(obj);            
            % Keep information about time input
            obj.pHasTimeInput = strcmpi(obj.TimeInputSource, 'Input port');
        end
        
        function setAssignmentResults(~,~,~,~)
            %Nothing to do in Simulink.
        end
        function setupDetectionManager(obj,detections)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker
            setupDetectionManager@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj,detections);
        end
        
        function track = allocateTrack(obj,sampleDetection)
            %Allocate the track.
            track = allocateTrack@trackerTOMHT(obj,sampleDetection);
            track.pIsInSimulink = true;
        end

        function out = getMaxNumInputDetections(obj)
            %In Simuklink forward the funciton call to AbstractSimulinkTracker.
            out = getMaxNumInputDetections@matlabshared.tracking.internal.fusion.AbstractSimulinkTracker(obj);
        end
        
        function detectables = getDetectableTrackIDsFromInput(obj,varargin)
            %Get detectable track ids from input in Simulink.
            detectables = getDetectableTrackIDsInSimulink(obj,varargin{:});
        end

        function costMatrix = getCostMatrixFromInput(obj,varargin)
            %Get costMatrix from input in Simulink.
            costMatrix = getCostMatrixinSimulink(obj,varargin{:}) ;
        end
        
        function validateInputsImpl(obj,~,varargin)
            % Validate inputs to the step method at initialization
            if strcmpi(obj.TimeInputSource, 'Input port')
                % Validate time input
                if numel(varargin) > 0
                    validateattributes(varargin{1},{'single','double'}, ...
                        {'real','finite','nonsparse','scalar','nonnegative'}, ...
                        mfilename, 'time')
                end
            end
        end
        
        function trs = getTrack(obj, inpTrs)
            % Pre-allocate memory
            % Note: the following will create a 1x0 struct array if there
            % are no live tracks, which is the desired output
            numTracks = numel(inpTrs);
            st = trackToStruct(obj.pTracksList{1},obj.TrackerIndex,obj.StateParameters,true);
            tr = optionalObjectAttributes(obj,st);
            track = optionalStateParameters(obj,tr);
            tracks = repmat(track, [numTracks, 1]);
            
            if  numTracks > 0
                % Populate the output array with information
                for i = 1:numTracks
                    tracks(i) = inpTrs(i);
                end
            end
            trs = sendToBus(obj, tracks, 1, track);
        end
        
        function info = getInfo(obj, infomation)
            sampleInfo = defaultInfo(obj);
            infoTrs = padInfoData(obj, infomation);
            info = sendToBus(obj, sampleInfo, 2, infoTrs);
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,prop) ;
            flag = flag || isPropInactiveInSimulink(obj,prop);            
            if ((~obj.AllBranchesOutputPort) && (obj.HasDetectableBranchIDsInput ...
                    || obj.HasCostMatrixInput))
                flag = flag | any(strcmp(prop, 'AllBranchesOutputPort'));
            end
            
            if ~strcmp(obj.OutputRepresentation,'Hypothesis') % No a Hypothesis
                flag = flag | strcmp(prop, 'HypothesesToOutput');
            end
        end
        
        function flag = isInputSizeMutableImpl(obj, index)
            % Return false if input size is not allowed to change while
            % system is running
            flag = true;
            if strcmpi(obj.TimeInputSource, 'Input port') && index == 2 % time is an input
                flag = false;
            end
            if obj.HasStateParametersInput && ...
                    isequal(index,(strcmpi(obj.TimeInputSource, 'Input port') + ...
                    obj.HasStateParametersInput + obj.HasCostMatrixInput + ...
                    obj.HasDetectableBranchIDsInput))
                flag = false;
            end
        end
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
            if strcmpi(obj.TimeInputSource, 'Input port')
                num = num + 1;
            end
            if obj.HasCostMatrixInput
                num = num + 1;
            end
            if obj.HasDetectableBranchIDsInput
                num = num + 1;
            end
            if obj.HasStateParametersInput
                num = num + 1;
            end
        end
        
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system
            num = 1;
            if obj.TentativeTracksOutputPort
                num = 2;
            end
            if obj.AllTracksOutputPort
                num = num + 1;
            end
            if obj.InfoOutputPort
                num = num + 1;
            end
            if (obj.AllBranchesOutputPort || obj.HasDetectableBranchIDsInput ...
                    || obj.HasCostMatrixInput)
                num = num + 1;
            end
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = getString(message('fusion:simulink:trackerTOMHT:TOMHTICON'));
        end
        
        function [name,varargout] = getInputNamesImpl(obj)
            % Return input port names for System block
            name = getString(message('fusion:simulink:trackerTOMHT:Detection'));
            varargout = {};
            if strcmpi(obj.TimeInputSource, 'Input port')
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:Prediction'))];
            end
            if obj.HasCostMatrixInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:CostMatrix'))];
            end
            if obj.HasDetectableBranchIDsInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:Detectable'))];
            end
            if obj.HasStateParametersInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:Param'))];
            end
        end
        
        function [name1, varargout] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = getString(message('fusion:simulink:trackerTOMHT:ConfirmedTrs'));
            varargout = {};
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:TentativeTrs'))];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:AllTrs'))];
            end
            if obj.InfoOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:Info'))];
            end
            if (obj.AllBranchesOutputPort || obj.HasCostMatrixInput ...
                    || obj.HasDetectableBranchIDsInput)
                
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerTOMHT:Branch'))];
            end
        end
        
        function [sz1,sz2,sz3,sz4,sz5] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [1 1];
            sz4 = [1 1];
            sz5 = [1 1];
        end
        
        function  varargout = getOutputDataTypeImpl(obj)
            if obj.InfoOutputPort
                [dtTracks, dtInfo] =  getBusDataTypes(obj);
            else
                dtTracks =  getBusDataTypes(obj);
            end
            varargout = {};
            varargout{1} = dtTracks;
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
            if obj.InfoOutputPort
                varargout = [varargout(:)' {dtInfo}];
            end
            if (obj.AllBranchesOutputPort || obj.HasDetectableBranchIDsInput ...
                    || obj.HasCostMatrixInput)
                varargout = [varargout(:)' {dtTracks}];
            end
        end
        
        function [cp1,cp2,cp3,cp4,cp5] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
            cp5 = false;
        end
        
        function [out1,out2,out3,out4,out5] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
        end
        
    end
    
    methods(Hidden)
        function infoStruct = defaultInfo(obj, maxDets, classToUse)
            num = obj.MaxNumTracks;
            if nargin == 1
                maxDets = obj.cDetectionManager.maxNumDetections;
                classToUse = obj.pClassToUse;
            end
            numMaxBranch = obj.MaxNumTracks*obj.MaxNumTrackBranches;
            pruningInfo = struct('BranchID',zeros(numMaxBranch, 1,'uint32'),...
                'PriorProbability', zeros(numMaxBranch,1),...
                'GlobalProbability', zeros(numMaxBranch,1),...
                'PrunedByProbability',false(numMaxBranch,1),...
                'PrunedByNScan' ,false(numMaxBranch,1),...
                'PrunedByNumBranches',false(numMaxBranch,1));
            infoStruct = struct(...
                'OOSMDetectionIndices',zeros(1,maxDets,'uint32'), ...
                'BranchIDsAtStepBeginning', zeros(num,1,'uint32'), ...
                'CostMatrix',zeros(num,maxDets,classToUse), ...
                'Assignments',zeros(num,2,'uint32'), ...
                'UnassignedTracks', zeros(num,1, 'uint32'), ...
                'UnassignedDetections', zeros(maxDets, 1, 'uint32'), ...
                'InitialBranchHistory', zeros(numMaxBranch,...
                3+(obj.MaxNumSensors*obj.MaxNumHistoryScans),'uint32'), ...
                'InitialBranchScores', zeros(numMaxBranch, 2), ...
                'KeptBranchHistory', zeros(numMaxBranch,...
                3+(obj.MaxNumSensors*obj.MaxNumHistoryScans), 'uint32'), ...
                'KeptBranchScores', zeros(numMaxBranch, 2), ...
                'Clusters', false(numMaxBranch, numMaxBranch), ...
                'TrackIncompatibility', false(numMaxBranch, numMaxBranch), ...
                'GlobalHypotheses', false(numMaxBranch, obj.MaxNumHypotheses), ...
                'GlobalHypScores', zeros(1, obj.MaxNumHypotheses), ...
                'DetailedPruningInfo', pruningInfo, ...
                'PrunedBranches', false(numMaxBranch,1), ...
                'GlobalBranchProbabilities', zeros(numMaxBranch, 1), ...
                'BranchesDeletedByPruning', zeros(numMaxBranch, 1, 'uint32'), ...
                'BranchIDsAtStepEnd', zeros(1, num, 'uint32'));
        end
        
        function info = padInfoData(obj,infoTrack)
            % Variable size data can not be logged in simulink hence
            % padding the data to make it fixed size.
            
            oosmDetInds = infoTrack.OOSMDetectionIndices; % Already fixed-size
            
            num = obj.MaxNumTracks;
            numMaxBranch = obj.MaxNumTracks*obj.MaxNumTrackBranches;
            trIDs = zeros(num,1,'like',infoTrack.BranchIDsAtStepBeginning);
            if ~isempty(infoTrack.BranchIDsAtStepBeginning)
                d1 = numel(infoTrack.BranchIDsAtStepBeginning);
                trIDs(1:d1)= infoTrack.BranchIDsAtStepBeginning;
            end
            
            costMat = zeros(num, obj.cDetectionManager.maxNumDetections, 'like', infoTrack.CostMatrix);
            if ~isempty(infoTrack.CostMatrix)
                d1 = size(infoTrack.CostMatrix,1);
                d2   = size(infoTrack.CostMatrix,2);
                costMat(1:d1,1:d2)= infoTrack.CostMatrix(1:d1,1:d2);
            end
            
            assgn = zeros(num,2,'like',infoTrack.Assignments);
            if ~isempty(infoTrack.Assignments)
                d1 = size(infoTrack.Assignments,1);
                assgn(1:d1,:) = infoTrack.Assignments;
            end
            
            unasTrs = zeros(num, 1, 'like', infoTrack.UnassignedTracks);
            if ~isempty(infoTrack.UnassignedTracks)
                d1 = numel(infoTrack.UnassignedTracks);
                unasTrs(1:d1) = infoTrack.UnassignedTracks;
            end
            
            unassgnDets = zeros(obj.cDetectionManager.maxNumDetections, 1, 'like', infoTrack.UnassignedDetections);
            if ~isempty(infoTrack.UnassignedDetections)
                d1 = numel(infoTrack.UnassignedDetections);
                unassgnDets(1:d1) = infoTrack.UnassignedDetections;
            end
            
            InitBranchHistory = zeros(numMaxBranch, 3+(obj.MaxNumSensors*obj.MaxNumHistoryScans), ...
                'like',infoTrack.InitialBranchHistory);
            if ~isempty(infoTrack.InitialBranchHistory)
                d1 = size(infoTrack.InitialBranchHistory, 1);
                d2 = size(infoTrack.InitialBranchHistory, 2);
                InitBranchHistory(1:d1,1:d2)= infoTrack.InitialBranchHistory(1:d1,1:d2);
            end
            
            InitBranchScores = zeros(numMaxBranch, 2, 'like', infoTrack.InitialBranchScores);
            if ~isempty(infoTrack.InitialBranchScores)
                d1 = size(infoTrack.InitialBranchScores,1);
                d2   = size(infoTrack.InitialBranchScores,2);
                InitBranchScores(1:d1,1:d2)= infoTrack.InitialBranchScores(1:d1,1:d2);
            end
            
            
            KeptHistory = zeros(numMaxBranch, 3+(obj.MaxNumSensors*obj.MaxNumHistoryScans), ...
                'like', infoTrack.KeptBranchHistory);
            if ~isempty(infoTrack.KeptBranchHistory)
                d1 = size(infoTrack.KeptBranchHistory,1);
                d2   = size(infoTrack.KeptBranchHistory,2);
                KeptHistory(1:d1,1:d2)= infoTrack.KeptBranchHistory(1:d1,1:d2);
            end
            
            KeptScores = zeros(numMaxBranch, 2, 'like', infoTrack.KeptBranchScores);
            if ~isempty(infoTrack.KeptBranchScores)
                d1 = size(infoTrack.KeptBranchScores,1);
                d2   = size(infoTrack.KeptBranchScores,2);
                KeptScores(1:d1,1:d2) = infoTrack.KeptBranchScores(1:d1,1:d2);
            end
            
            
            clst = false(numMaxBranch, numMaxBranch);
            if ~isempty(infoTrack.Clusters)
                d1 = size(infoTrack.Clusters,1);
                d2   = size(infoTrack.Clusters,2);
                clst(1:d1,1:d2)= infoTrack.Clusters(1:d1,1:d2);
            end
            
            trsIncompatibility = false(numMaxBranch, numMaxBranch);
            if ~isempty(infoTrack.TrackIncompatibility)
                d1 = size(infoTrack.TrackIncompatibility,1);
                d2   = size(infoTrack.TrackIncompatibility,2);
                trsIncompatibility(1:d1,1:d2)= infoTrack.TrackIncompatibility(1:d1,1:d2);
            end
            
            globalHyp = false(numMaxBranch, obj.MaxNumHypotheses);
            if ~isempty(infoTrack.GlobalHypotheses)
                d1 = size(infoTrack.GlobalHypotheses,1);
                d2 = size(infoTrack.GlobalHypotheses,2);
                globalHyp(1:d1,1:d2)= infoTrack.GlobalHypotheses(1:d1,1:d2);
            end
            
            
            globalHypScrs = zeros(1, obj.MaxNumHypotheses, 'like', infoTrack.GlobalHypScores);
            if ~isempty(infoTrack.GlobalHypScores)
                d1 = numel(infoTrack.GlobalHypScores);
                globalHypScrs(1:d1) = infoTrack.GlobalHypScores;
            end
            
            prunedBranch = false(numMaxBranch, 1);
            if ~isempty(infoTrack.PrunedBranches)
                d1 = numel(infoTrack.PrunedBranches);
                prunedBranch(1:d1) = infoTrack.PrunedBranches;
            end
            
            globalBranchProb = zeros(numMaxBranch, 1, 'like', infoTrack.GlobalBranchProbabilities);
            if ~isempty(infoTrack.GlobalBranchProbabilities)
                d1 = numel(infoTrack.GlobalBranchProbabilities);
                globalBranchProb(1:d1) = infoTrack.GlobalBranchProbabilities;
            end
            
            deletedBranch = zeros(numMaxBranch, 1, 'like', infoTrack.BranchesDeletedByPruning);
            if ~isempty(infoTrack.BranchesDeletedByPruning)
                d1 = numel(infoTrack.BranchesDeletedByPruning);
                deletedBranch(1:d1) = infoTrack.BranchesDeletedByPruning;
            end
            
            brachIDsEnd = zeros(1, num, 'like', infoTrack.BranchIDsAtStepEnd);
            if ~isempty(infoTrack.BranchIDsAtStepEnd)
                d1 = numel(infoTrack.BranchIDsAtStepEnd);
                brachIDsEnd(1:d1) = infoTrack.BranchIDsAtStepEnd;
            end
            
            brachIDs = zeros(numMaxBranch, 1, 'like', infoTrack.DetailedPruningInfo.BranchID);
            if ~isempty(infoTrack.DetailedPruningInfo.BranchID)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.BranchID);
                brachIDs(1:numTrsInitial) = infoTrack.DetailedPruningInfo.BranchID;
            end
            
            priorProb = zeros(numMaxBranch, 1, 'like', infoTrack.DetailedPruningInfo.PriorProbability);
            if ~isempty(infoTrack.DetailedPruningInfo.PriorProbability)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.PriorProbability);
                priorProb(1:numTrsInitial) = infoTrack.DetailedPruningInfo.PriorProbability;
            end
            
            glbProb = zeros(numMaxBranch, 1, 'like', infoTrack.DetailedPruningInfo.GlobalProbability);
            if ~isempty(infoTrack.DetailedPruningInfo.GlobalProbability)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.GlobalProbability);
                glbProb(1:numTrsInitial) = infoTrack.DetailedPruningInfo.GlobalProbability;
            end
            
            prunedByProb = false(numMaxBranch, 1);
            if ~isempty(infoTrack.DetailedPruningInfo.PrunedByProbability)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.PrunedByProbability);
                prunedByProb(1:numTrsInitial) = infoTrack.DetailedPruningInfo.PrunedByProbability;
            end
            
            prunedByScan = false(numMaxBranch, 1);
            if ~isempty(infoTrack.DetailedPruningInfo.PrunedByNScan)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.PrunedByNScan);
                prunedByScan(1:numTrsInitial) = infoTrack.DetailedPruningInfo.PrunedByNScan;
            end
            
            prunedByBranch = false(numMaxBranch, 1);
            if ~isempty(infoTrack.DetailedPruningInfo.PrunedByNumBranches)
                numTrsInitial = numel(infoTrack.DetailedPruningInfo.PrunedByNumBranches);
                prunedByBranch(1:numTrsInitial) = infoTrack.DetailedPruningInfo.PrunedByNumBranches;
            end
            
            pruningInfo = struct('BranchID',brachIDs,...
                'PriorProbability', priorProb,...
                'GlobalProbability', glbProb,...
                'PrunedByProbability',prunedByProb,...
                'PrunedByNScan' ,prunedByScan,...
                'PrunedByNumBranches',prunedByBranch);
            
            info = struct(...
                'OOSMDetectionIndices', oosmDetInds, ...
                'BranchIDsAtStepBeginning', trIDs, ...
                'CostMatrix', costMat, ...
                'Assignments', assgn, ...
                'UnassignedTracks', unasTrs, ...
                'UnassignedDetections', unassgnDets, ...
                'InitialBranchHistory', InitBranchHistory, ...
                'InitialBranchScores', InitBranchScores, ...
                'KeptBranchHistory', KeptHistory, ...
                'KeptBranchScores', KeptScores, ...
                'Clusters', clst, ...
                'TrackIncompatibility', trsIncompatibility, ...
                'GlobalHypotheses', globalHyp, ...
                'GlobalHypScores', globalHypScrs, ...
                'DetailedPruningInfo', pruningInfo, ...
                'PrunedBranches', prunedBranch, ...
                'GlobalBranchProbabilities', globalBranchProb, ...
                'BranchesDeletedByPruning', deletedBranch, ...
                'BranchIDsAtStepEnd', brachIDsEnd);
        end
    end
    
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:tomhtTrackerTitle', ...
                'Text',	 'fusion:block:tomhtTrackerDesc');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            
            absTrkrSection = matlabshared.tracking.internal.fusion.AbstractSimulinkTracker.getBusPropertyGroups;
            
            propRepresentationList = {'OutputRepresentation', 'HypothesesToOutput'};
            RepresenatationSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','Output',propRepresentationList);
            propCapacityList = {'TrackerIndex', 'FilterInitializationFcn', ...
                'AssignmentThreshold', 'MaxNumTracks', 'MaxNumSensors', 'OOSMHandling', ...
                'StateParametersSimulink','HasStateParametersInput'};
            CapacitySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','',propCapacityList);
            CapacityGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerTOMHT:GroupCapacitySection')), ...
                'Sections', [CapacitySection, RepresenatationSection]);
            
            propPruningList = {'MinBranchProbability', 'NScanPruning'};
            PruningSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','Pruning',propPruningList);
            propHypothesesList = {'MaxNumHypotheses', 'MaxNumHistoryScans',...
                'MaxNumTrackBranches'};
            HypothesesSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','',propHypothesesList);
            HypothesesGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerTOMHT:GroupHypothesesSection')), ...
                'Sections', [HypothesesSection, PruningSection]);
            
            propScoreList = {'ConfirmationThreshold', 'DeletionThreshold',...
                'DetectionProbability', 'FalseAlarmRate', 'Volume', 'Beta'};
            ScoreSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','',propScoreList);
            ScoreGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerTOMHT:GroupScoreSection')), ...
                'Sections', ScoreSection);
            
            busList = absTrkrSection.PropertyList;
            redirectList = {1 , 'SimulinkBusPropagation'; 2, 'SimulinkBusPropagation'};
            portsList = [busList(:)',{'BusName2Source'},{'BusName2'}];
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','OutputPortSettings', portsList, redirectList);
            
            propIOList = {'TimeInputSource', 'HasCostMatrixInput', 'HasDetectableBranchIDsInput', ...
                'TentativeTracksOutputPort', 'AllTracksOutputPort', 'InfoOutputPort', 'AllBranchesOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerTOMHT','InputsOutputs', propIOList);
            ioGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerTOMHT:GroupIO')), ...
                'Sections', [ioSection,portsSection]);
            
            groups = [CapacityGroup, HypothesesGroup, ScoreGroup, ioGroup];
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end
end
