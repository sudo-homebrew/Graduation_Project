classdef OOSMRetrodictionGNN < matlab.System
    %OOSMRetrodictionGNN OOSM handler for retrodiction using GNN assignment
    % handler = OOSMRetrodictionGNN() returns an OOSM handler that
    % implements the retrodiction technique for a GNN tracker.
    %
    % handler = OOSMRetrodictionGNN('Name', value) by specifying its
    % properties as name-value pair arguments. Unspecified properties have
    % default values. See the list of properties below.
    %
    % OOSMRetrodictionGNN properties:
    %   MaxNumOOSMSteps      - Maximum number of retrodiction steps
    %   Tracks               - A list of tracks
    %   DetectionManager     - A detection manager
    %   Assigner             - A GNN assigner object
    %   Volume               - Volume
    %   DetectionProbability - Detection probability
    %   FalseAlarmRate       - False alarm rate
    %   HasCostMatrixInput   - True if cost matrix input is used
    %   UsedSensors          - List of used sensors
    %   NumLiveTracks        - Number of live tracks
    %   IsInSimulink         - True if the object is used in Simulink
    %
    % Step method syntax:
    %   oosmInfo = stepImpl(obj, currentTime)
    %   updates the OOSMRetrodictionGNN. You must use this syntax if
    %   HasCostMatrixInput is false.
    %   
    %   oosmInfo = stepImpl(..., costMatrix), additionally, lets you pass a
    %   cost matrix to the input. You must use this syntax if
    %   HasCostMatrixInput is true.
    %
    %   The OOSMRetrodictionGNN:
    %     1. Retrieves detections from the DetectionManager.
    %     2. Retrodicts the track in Tracks to the detection times.
    %     3. Calcualtes the cost of assigning detections to tracks.
    %     4. Assigns the detections to the tracks .
    %     5. Retro-corrects the tracks with the assigned detections. 
    
    % Copyright 2021 The MathWorks, Inc.
    %#codegen
    
    % The following properties are set on construction by the tracker
    
    properties
        %Tracks   Tracks list shared with the tracker
        Tracks
        
        %DetectionManager  Detection manager shared with the tracker
        DetectionManager (1, 1) matlabshared.tracking.internal.fusion.AbstractDetectionManager = ...
            matlabshared.tracking.internal.fusion.MATLABDetectionManager
        
        %Assigner   Assigner shared with the tracker
        Assigner (1, 1) matlabshared.tracking.internal.fusion.AssignerGNN
    end
    
    properties(Nontunable) 
        %Volume Volume same as tracker
        Volume = 1
        
        %DetectionProbability  Detection probability same as the tracker
        DetectionProbability = 0.9
        
        %FalseAlarmRate  False alarm rate same as the tracker
        FalseAlarmRate = 1e-6
        
        %MaxNumOOSMSteps Maximum number of retrodiction step
        MaxNumOOSMSteps (1, 1) {mustBePositive, mustBeInteger} = 3
        
        %HasCostMatrixInput Provide cost matrix as an input
        HasCostMatrixInput (1, 1) logical = false

        %HasMeasurementParameters Defines if detections have measurement
        % parameters
        HasMeasurementParameters (1, 1) logical = false
        
        %IsInSimulink True if in Simulink
        IsInSimulink (1, 1) logical = false

        %MaxNumSensors Maximum number of sensors
        MaxNumSensors (1, 1) {mustBePositive, mustBeInteger} = 20;
    end
    
    properties
        %UsedSensors   Sensor indices used by the tracker
        UsedSensors
        
        %NumLiveTracks Number of live tracks in this step
        NumLiveTracks (1,1) int32 {mustBeNonnegative, mustBeInteger} = 0
    end
    
    properties(Access = protected, Nontunable)
        % Saves the max number of tracks
        pMaxNumTracks (1,1) int32
    end
    
    properties(Access = {?fusion.internal.OOSMRetrodictionGNN, ...
            ?matlab.unittest.TestCase})
        % Keeps history of all timestamps
        pTimeStampHistory
    end

    methods
        function obj = OOSMRetrodictionGNN(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.pMaxNumTracks = obj.IntNumel(obj.Tracks);
            
            obj.pTimeStampHistory = zeros(1,obj.MaxNumOOSMSteps,'like',obj.Tracks{1}.State);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.pTimeStampHistory = zeros(1,obj.MaxNumOOSMSteps,'like',obj.Tracks{1}.State);
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            if isfield(s,'pTimeStampHistory') && ~isempty(s.pTimeStampHistory)
                obj.pTimeStampHistory = s.pTimeStampHistory;
                s = rmfield(s,'pTimeStampHistory');
            end

            if isfield(s, 'pMaxNumTracks') && ~isempty(s.pMaxNumTracks)
                obj.pMaxNumTracks = s.pMaxNumTracks;
                s = rmfield(s,'pMaxNumTracks');
            end

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            if ~isempty(obj.pMaxNumTracks)
                s.pMaxNumTracks = obj.pMaxNumTracks;
            end

            if ~isempty(obj.pTimeStampHistory)
                s.pTimeStampHistory = obj.pTimeStampHistory;
            end
        end
        
        function oosmInfo = stepImpl(obj, currentTime, varargin)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % Find old detections
            detTimes = detectionTimes(obj.DetectionManager);
            oldestTime = min(obj.pTimeStampHistory);
            oldDets = detTimes < oldestTime;
            
            % Get detections once
            dets = detections(obj.DetectionManager);
            
            % Calculate distance to OOSM detections
            if obj.HasCostMatrixInput
                allCosts = varargin{1};
                cost = allCosts(1:obj.NumLiveTracks, :);
            else
                cost = calculateCost(obj,dets,oldDets);
                cost(cost > obj.Assigner.Assignment(1)) = inf;
            end
            
            % Assign OOSMs to tracks
            [assignments, unassignedDetections] = assign(obj,cost,detTimes);
            
            % Retrocorrect the tracks with assigned OOSMs
            retroCorrectTracks(obj,dets,assignments);
            
            % Update timestamp history
            keepTimestampHistory(obj, currentTime);
            
            dc = uint32(reshape(find(oldDets),1,[]));
            assn = fixTrackIDs(obj,assignments);
            if ~obj.IsInSimulink
                oosmInfo = struct(...
                    'DiscardedDetections', dc, ...
                    'CostMatrix', cost, ...
                    'Assignments', assn, ...
                    'UnassignedDetections', unassignedDetections ...
                    );
            else
                mnd = maxNumDetections(obj.DetectionManager);
                oosmInfo = struct(...
                    'DiscardedDetections', padIt(dc,1,mnd), ...
                    'CostMatrix', padIt(cost,numel(obj.Tracks),mnd), ...
                    'Assignments', padIt(assn,numel(obj.Tracks),2), ...
                    'UnassignedDetections', padIt(unassignedDetections,mnd,1) ...
                    );
            end
        end
        
        function keepTimestampHistory(obj, currentTime)
            % Maintains the update times at each step in MaxNumOOSMSteps
            % Timestamp history is a buffer that gets rewritten - the
            % oldest timestamp is replaced by the newest
            [~,ind] = min(obj.pTimeStampHistory);
            obj.pTimeStampHistory(ind) = currentTime;
        end

        function cost = calculateCost(obj,dets,oldDets)
            % Calculate cost of assignment for all the tracks
            % We only calculate cost for the detections that are not too
            % old beyond the history of the OOSM handler
            
            n = obj.NumLiveTracks;
            assert(n <= obj.pMaxNumTracks);
            m = numel(dets);
            cost = zeros(n, m, 'like', obj.Tracks{1}.State);
            
            % Redefine OOSM detections as the ones that are not too old
            isoosm = (isOOSM(obj.DetectionManager) & ~oldDets);
            
            % Calcualte cost using retroDistance on the track
            for trkInd = 1:obj.NumLiveTracks
                track = obj.Tracks{trkInd};
                trackCost = retroDistance(track,dets,find(isoosm),obj.HasMeasurementParameters);
                cost(trkInd,isoosm) = trackCost;
            end
        end
        
        function [allAssignments,unassignedDetections] = assign(obj, cost, detTimes)
            % Assign OOSMs to tracks
            % Report all the assignments and all the unassigend OOSMs
            
            numDets = size(cost,2);
            allDetInds = uint32(1:numDets)';
            isDetectionAssigned = true(1,numDets);
            inOOSMRange = detTimes <= max(obj.pTimeStampHistory) & detTimes >= min(obj.pTimeStampHistory);
            if isempty(cost) || ~any(inOOSMRange)
                allAssignments = zeros(0,2,'uint32');
                % If there are no tracks, cost is empty and all OOSMs
                % should be unassigned
                if any(inOOSMRange) && max(obj.pTimeStampHistory) > 0
                    isDetectionAssigned(1,inOOSMRange) = false;
                end
            else
                [numTracks, numDetections] = obj.IntSize2D(cost);
                maxNumAssignments = min(numTracks*obj.MaxNumOOSMSteps*obj.MaxNumSensors,numDetections);
                allAssignments = zeros(maxNumAssignments,2,'uint32');
                m = max(obj.pTimeStampHistory);
                historyTimes = sort([obj.pTimeStampHistory m+eps(m)]);
                inOOSMRange = detTimes <= historyTimes(end) & detTimes >= historyTimes(1);
                isDetectionAssigned(1,inOOSMRange) = false;
                % Assign needs to take the detections per update / scan.
                numPrevAssigned = obj.IntZero;
                for i = obj.IntOne:obj.IntNumel(historyTimes) - obj.IntOne
                    inInterval = detTimes >= historyTimes(i) & detTimes < historyTimes(i+1);
                    if any(inInterval)
                        intervalInds = obj.IntFind(inInterval);
                        assignments = step(obj.Assigner, cost(:,inInterval));
                        assignments(:,2) = intervalInds(1,assignments(:,2)); % Get absolute detection indices
                        isDetectionAssigned(1,assignments(:,2)) = true;
                        sz = obj.IntSize2D(assignments);
                        numCurrentlyAssigned = sz(1);
                        allAssignments(numPrevAssigned+(1:numCurrentlyAssigned),:) = assignments;
                        numPrevAssigned = numPrevAssigned + numCurrentlyAssigned;
                    end
                end
                allAssignments = allAssignments(1:numPrevAssigned,:);
            end
            unassignedDetections = allDetInds(~isDetectionAssigned,1);
        end
        
        function retroCorrectTracks(obj,dets,assignments)
            % Correct all the tracks with assigned OOSMs
            if ~isempty(assignments)
                UniqueTracks = unique(assignments(:, 1));
                for i = 1:obj.IntNumel(UniqueTracks)
                    detectionsForCorrect = assignments(assignments(:, 1) == UniqueTracks(i), 2);
                    coder.varsize('detectionsForCorrect',[obj.MaxNumSensors*obj.MaxNumOOSMSteps,1],[1 0]);
                    assert(numel(detectionsForCorrect) <= obj.MaxNumSensors*obj.MaxNumOOSMSteps);
                    trackDetections = extractDetectionsForTrack(obj,dets,detectionsForCorrect);
                    if ~isempty(detectionsForCorrect)
                        retroCorrect(obj.Tracks{UniqueTracks(i)}, ...
                            trackDetections, obj.UsedSensors, obj.Volume, ...
                            obj.DetectionProbability, obj.FalseAlarmRate);
                    end
                end
            end
        end
        
        function trackDetections = extractDetectionsForTrack(obj,dets,detectionsForCorrect)
            % Get the list of detections assigned to the track
            numTrackDetections = obj.IntNumel(detectionsForCorrect);
            assert(numTrackDetections <= maxNumDetections(obj.DetectionManager));
            trackDetections = repmat({dets{detectionsForCorrect(1)}}, [numTrackDetections, 1]);
            for i = 2:numTrackDetections
                trackDetections{i} = dets{detectionsForCorrect(i)};
            end
        end
        
        function assignments = fixTrackIDs(obj, assignments)
            % Get the correct track ID instead of the track index in
            % assignments
            sz = obj.IntSize2D(assignments);
            numIDs = sz(1);
            for i = obj.IntOne:numIDs
                idx = obj.IntIndex(assignments(i,1));
                assignments(i,1) = obj.Tracks{idx}.TrackID;
            end
        end
    end
    
    methods(Hidden,Static)
        function [nRow, nCol] = IntSize2D(in)
            [nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(in);
        end
        function idx = IntZero()
            idx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
        end
        function idx = IntOne()
            idx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
        end
        function idx = IntIndex(idx)
            idx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(idx);
        end
        function n = IntNumel(in)
            n = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntNumel(in);
        end
        function n = IntLogicalSum(in)
            n = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntLogicalSum(in);
        end
        function n = IntFind(in,varargin)
            n = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntFind(in,varargin{:});
        end
    end
end

function out = padIt(in, sz1, sz2)
classToUse = class(in);
out = zeros(sz1, sz2, classToUse);
out(1:size(in,1),1:size(in,2)) = in;
end