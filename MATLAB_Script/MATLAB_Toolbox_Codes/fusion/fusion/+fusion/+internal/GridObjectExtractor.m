classdef GridObjectExtractor < matlab.System
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % obj = fusion.internal.GridObjectExtractor creates an object track
    % extractor
    %
    % obj = fusion.internal.GridObjectExtractor(Name,value) allows
    % specifying properties using N/V pairs.
    %
    % [confTracks, tentTracks, allTracks] = step(obj, dynamicMap, time)
    % generates a list of objectTrack outputs from the dynamic map,
    % dynamicMap.
    %
    % Cells which are declared dynamic are considered for extraction of
    % objects.
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    % Prediction related properties
    properties (Nontunable)
        StateTransitionFcn
        StateTransitionJacobianFcn
        HasAdditiveProcessNoise
    end
    
    properties
        ProcessNoise
    end
    
    properties (Nontunable)
        ExtractorIndex = uint32(1)
    end
    
    properties
        StateParameters
    end
    
    properties (Nontunable)
        AssignmentThreshold = 100;
    end
    
    % Extraction of tracks.
    properties (Nontunable)
        % A function which initializes the track using the following
        % signature
        %
        % track = TrackInitializationFcn(gridState, associatedCells)
        TrackInitializationFcn
        
        % A function which updates the track using the following signature
        %
        % track = TrackUpdateFcn(track, associatedCells)
        TrackUpdateFcn
        
        % A function which computes the distance using the following
        % signature lhood = TrackDistanceFcn(map, track);
        TrackDistanceFcn
    end
    
    properties (Nontunable)
        ConfirmationThreshold = [2 3]
        DeletionThreshold = [4 5];
    end
    
    % Storing the list of objectTracks and their logics
    properties (Access = {?fusion.internal.GridObjectExtractor,...
            ?matlab.unittest.TestCase})
        pTrackList
        pTrackLogics
        pNumLiveTracks
        pLastTrackID = uint32(0);
        pConfirmationFlags;
    end
    
    properties (Nontunable, Access = protected)
        pClassToUse
    end
    
    properties (Nontunable)
        % MaxNumTracks - Maximum number of tracks maintained by the object
        % extractor.
        %
        % Default: 100
        MaxNumTracks = 100;
    end
    
    % A function to perform clustering.
    properties (Nontunable)
        % This function should perform the following operation
        %
        % idx = ClusteringFcn(dynamicGridCells)
        ClusteringFcn
    end
    
    properties (Access = protected)
        % pSampleCellInformation stores the cell information structure
        % provided by the dynamic map.
        pSampleCellInformation
        
        % Last the time extractor was updated.
        pLastTime
    end
    
    properties (Dependent)
        NumTracks
        NumConfirmedTracks
    end
    
    methods
        function val = get.NumTracks(obj)
            if coder.internal.is_defined(obj.pTrackList)
                val = obj.pNumLiveTracks;
            else
                val = 0;
            end
        end
        
        function val = get.NumConfirmedTracks(obj)
            if coder.internal.is_defined(obj.pTrackList)
                val = sum(obj.pConfirmationFlags(1:obj.pNumLiveTracks));
            else
                val = 0;
            end
        end
    end
    
    methods
        function obj = GridObjectExtractor(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end
    
    methods
        function set.TrackInitializationFcn(obj,val)
            if isa(val,'function_handle')
                obj.TrackInitializationFcn = val;
            elseif isa(val,'char') || isa(val,'string')
                obj.TrackInitializationFcn = str2func(val);
            end
        end
        
        function set.TrackUpdateFcn(obj,val)
            if isa(val,'function_handle')
                obj.TrackUpdateFcn = val;
            elseif isa(val,'char') || isa(val,'string')
                obj.TrackUpdateFcn = str2func(val);
            end
        end
        
        function set.TrackDistanceFcn(obj,val)
            if isa(val,'function_handle')
                obj.TrackDistanceFcn = val;
            elseif isa(val,'char') || isa(val,'string')
                obj.TrackDistanceFcn = str2func(val);
            end
        end
        
        function set.StateTransitionFcn(obj,val)
            if isa(val,'function_handle')
                obj.StateTransitionFcn = val;
            elseif isa(val,'char') || isa(val,'string')
                obj.StateTransitionFcn = str2func(val);
            end
        end
        
        function set.StateTransitionJacobianFcn(obj,val)
            if isa(val,'function_handle')
                obj.StateTransitionJacobianFcn = val;
            elseif isa(val,'char') || isa(val,'string')
                obj.StateTransitionJacobianFcn = str2func(val);
            end
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, map, varargin)
            % Assign the track list and corresponding track logics
            
            % Sample cell information from the map
            if ~coder.internal.is_defined(obj.pSampleCellInformation)
                obj.pSampleCellInformation = getSampleCell(map);
            end
            
            % Create track using user-provided TrackInitializationFcn
            initTrack = obj.TrackInitializationFcn(obj.pSampleCellInformation);
            
            % Validate that the track is compatible with state transition
            % properties.
            validateTrack(obj, initTrack);
            
            % SampleTrack using information from extractor as well as the
            % track.
            sampleTrack = createSampleTrack(obj, initTrack);
            
            % Create the track list
            obj.pTrackList = repmat(sampleTrack, obj.MaxNumTracks, 1);
            
            % List of confirmation flags
            obj.pConfirmationFlags = false(obj.MaxNumTracks,1);
            
            % Current number of tracks (all tracks)
            obj.pNumLiveTracks = 0;
            
            % Last update time
            obj.pLastTime = cast(-eps,obj.pClassToUse);
            
            % Instantiate all track logics
            trkLogics = cell(obj.MaxNumTracks,1);
            for i = 1:obj.MaxNumTracks
                trkLogics{i} = trackHistoryLogic(...
                    'ConfirmationThreshold', obj.ConfirmationThreshold, ...
                    'DeletionThreshold', obj.DeletionThreshold);
            end
            obj.pTrackLogics = trkLogics;
        end
    end
    
    methods (Access = protected)
        function validateTrack(obj, initTrack)
            % Validate that the StateTransitionFcn etc. are compatible with
            % the state of the track
            coder.internal.assert(isa(initTrack,'objectTrack') || isa(initTrack,'struct') && isscalar(initTrack),'fusion:GridTracker:InvalidTrackInitializationFcn');
            m = initTrack.State;
            P = initTrack.StateCovariance;
            dT = ones(1,'like',m);
            fusion.internal.gaussEKFilter.predict(m, P, ...
                obj.ProcessNoise,...
                obj.StateTransitionFcn,...
                obj.StateTransitionJacobianFcn,....
                obj.HasAdditiveProcessNoise,...
                dT);
            
            % Validate that the track can be updated with the update
            % function and produce a state of the same size
            updatedTrack = obj.TrackUpdateFcn(initTrack,obj.pSampleCellInformation);
            coder.internal.assert(isa(updatedTrack,'objectTrack') || isa(updatedTrack,'struct') && isscalar(updatedTrack),'fusion:GridTracker:InvalidTrackUpdateFcn');
            coder.internal.assert(numel(updatedTrack.State) == numel(m),'fusion:GridTracker:InvalidTrackStateSize',numel(m));
        end
        
        function sampleTrack = createSampleTrack(obj, initTrack)
            % Create sample track from information captured in the object
            % as well as the track supplied by the initialization function.
            
            state = initTrack.State;
            n = numel(state);
            classToUse = class(state);
            
            if ~coder.internal.is_defined(obj.pClassToUse)
                obj.pClassToUse = classToUse;
            end
            
            objAttributes = initTrack.ObjectAttributes;
            
            histLength = max(obj.ConfirmationThreshold(2),obj.DeletionThreshold(2));
            
            sampleObjectTrack = objectTrack('TrackID',1,...
                'State',zeros(n,1,classToUse),...
                'StateCovariance',eye(n,classToUse),...
                'SourceIndex',obj.ExtractorIndex,...
                'Age',uint32(0),...
                'StateParameters',obj.StateParameters,....
                'ObjectClassID',0,...
                'TrackLogic','History',...
                'TrackLogicState',false(1,histLength),...
                'ObjectAttributes',objAttributes,...
                'IsConfirmed',false,...
                'IsCoasted',false...
                );
            
            if coder.target('MATLAB')
                % In MATLAB create an array of objectTrack
                sampleTrack = sampleObjectTrack;
            else
                % In codegen create an array of structs
                sampleTrack = toStruct(sampleObjectTrack);
            end
        end
    end
    
    methods (Access = protected)
        function [confTracks, tentTracks, allTracks] = stepImpl(obj, map, time)
            % Predict existing tracks to current time
            t = cast(time,'like',obj.pLastTime);
            predictTracks(obj, t);
            
            % Associated dynamic grid with predicted tracks.
            [assignedDynamicCells, assignedTrackIndex, ...
                unassignedTrackIndex, unassignedDynamicCells] = computeAssignment(obj, map);
            
            % Using compute assignments, correct assigned tracks with their
            % respect assigned dynamic cells
            updateAssignedTracks(obj, assignedDynamicCells, assignedTrackIndex);
            
            % Update unassigned tracks by coasting
            updateUnassignedTracks(obj, unassignedTrackIndex);
            
            % Initiate new tracks from unassigned dynamic cells
            initiateNewTracks(obj, unassignedDynamicCells);
            
            % Delete tracks which are coasted for long time using the Track
            % Logic.
            deleteTracks(obj);
            
            % Extract all tracks
            [confTracks, tentTracks, allTracks] = extractOutputs(obj);
            
            obj.pLastTime = t;
        end
    end
    
    methods 
        function predictedTracks = predictTracksToTime(obj, dT, trkID, withCovariance)
            switch trkID
                case -1
                    idx = 1:obj.pNumLiveTracks;
                case -2
                    idx = find(obj.pConfirmationFlags(1:obj.pNumLiveTracks));
                case -3
                    idx = find(~obj.pConfirmationFlags(1:obj.pNumLiveTracks));
                otherwise
                    if coder.target('MATLAB')
                        allIDs = vertcat(obj.pTrackList(1:obj.pNumLiveTracks).TrackID);
                    else
                        allIDs = zeros(obj.pNumLiveTracks,1,'uint32');
                        for i = 1:obj.pNumLiveTracks
                            allIDs(i) = obj.pTrackList(i).TrackID;
                        end
                    end
                    idx = find(allIDs == trkID);
                    coder.internal.assert(~isempty(idx),'fusion:GridTracker:TrackUnavailable',trkID)
            end

            % predict without covariance
            if withCovariance
                predictedTracks = predictTracksWithCov(obj, dT, idx);
            else
                predictedTracks = predictTracksWithoutCov(obj, dT, idx);
            end
        end
        
        function predictedTracks = predictTracksWithoutCov(obj, dT, idx)
            tracks = obj.pTrackList(idx);
            if ~isempty(tracks)
                if coder.target('MATLAB')
                    m = horzcat(tracks.State);
                else
                    classToUse = class(tracks(1).State);
                    n = numel(tracks(1).State);
                    m = zeros(n,obj.pNumLiveTracks,classToUse);
                    for i = 1:obj.pNumLiveTracks
                        m(:,i) = tracks(i).State;
                    end
                end
                f = obj.StateTransitionFcn;
                if obj.HasAdditiveProcessNoise
                    m = f(m, dT);
                else
                    m = f(m,zeros(1,class(m)),dT);
                end
                for i = 1:numel(tracks)
                    tracks(i).State = m(:,i);
                    tracks(i).UpdateTime = tracks(i).UpdateTime + dT;
                end
            end
            predictedTracks = tracks;
        end
        
        function predictedTracks = predictTracksWithCov(obj, dT, idx)
            tracks = obj.pTrackList(idx);
            if ~isempty(tracks)
                if coder.target('MATLAB')
                    m = horzcat(tracks.State);
                    P = cat(3,tracks.StateCovariance);
                else
                    classToUse = class(tracks(1).State);
                    n = numel(tracks(1).State);
                    m = zeros(n,obj.pNumLiveTracks,classToUse);
                    P = zeros(n,n,obj.pNumLiveTracks,classToUse);
                    for i = 1:obj.pNumLiveTracks
                        m(:,i) = tracks(i).State;
                        P(:,:,i) = tracks(i).StateCovariance;
                    end
                end
                Q = obj.ProcessNoise;
                f = obj.StateTransitionFcn;
                df = obj.StateTransitionJacobianFcn;
                hasAPN = obj.HasAdditiveProcessNoise;
                [m, P] = fusion.internal.gaussEKFilter.predict(m, P, Q, f, df, hasAPN, dT);
                for i = 1:numel(tracks)
                    tracks(i).State = m(:,i);
                    tracks(i).StateCovariance = P(:,:,i);
                    tracks(i).UpdateTime = tracks(i).UpdateTime + dT;
                end
            end
            predictedTracks = tracks;
        end
    end
    
    methods (Access = {?fusion.internal.GridObjectExtractor,...
            ?matlab.unittest.TestCase})
        
        function tracks = predictTracks(obj, time)
            dT = time - obj.pLastTime;
            
            if obj.pNumLiveTracks > 0 && dT > 0
                tracks = obj.pTrackList(1:obj.pNumLiveTracks);
                n = numel(tracks(1).State);
                if coder.target('MATLAB')
                    m = horzcat(tracks.State);
                    P = cat(3,tracks.StateCovariance);
                else
                    classToUse = class(tracks(1).State);
                    m = zeros(n,obj.pNumLiveTracks,classToUse);
                    P = zeros(n,n,obj.pNumLiveTracks,classToUse);
                    for i = 1:obj.pNumLiveTracks
                        m(:,i) = tracks(i).State;
                        P(:,:,i) = tracks(i).StateCovariance;
                    end
                end
                Q = obj.ProcessNoise;
                f = obj.StateTransitionFcn;
                df = obj.StateTransitionJacobianFcn;
                hasAPN = obj.HasAdditiveProcessNoise;
                [m, P] = fusion.internal.gaussEKFilter.predict(m, P, Q, f, df, hasAPN, dT);
                for i = 1:numel(tracks)
                    tracks(i).State = m(1:n,i);
                    tracks(i).StateCovariance = P(1:n,1:n,i);
                    tracks(i).UpdateTime = time;
                end
                obj.pTrackList(1:obj.pNumLiveTracks) = tracks;
            end
        end
        
        function [assignedGridCellArray, assignedTrackIndex, unassignedTrackIndex, unassignedGridCells] = computeAssignment(obj, map)
            currentDistance = obj.AssignmentThreshold*ones(map.GridSize,'like',obj.pSampleCellInformation.States);
            currentLabels = zeros(map.GridSize,'like',obj.pSampleCellInformation.States);
            
            % Nearest neighbor approach for assignment
            for i = 1:obj.pNumLiveTracks
                thisTrk = obj.pTrackList(i);
                d = obj.TrackDistanceFcn(map, thisTrk);
                toUpdate = d < currentDistance;
                currentLabels(toUpdate) = i;
                currentDistance(toUpdate) = d(toUpdate);
            end
            
            % Information about all assigned cells.
            allCellInfo = repmat(obj.pSampleCellInformation, obj.pNumLiveTracks, 1);
            
            % Indices of all tracks
            allTrackIndex = cast(1:obj.pNumLiveTracks,'uint32');
            
            % Flag if the track is assigned
            isAssigned = false(obj.pNumLiveTracks,1);
            
            for i = 1:obj.pNumLiveTracks
                assignedCellIndex = currentLabels == i;
                if any(assignedCellIndex(:))
                    isAssigned(i) = true;
                    allCellInfo(i) = getCells(map, find(assignedCellIndex));
                end
            end
            
            assignedGridCellArray = allCellInfo(isAssigned);
            assignedTrackIndex = allTrackIndex(isAssigned);
            unassignedTrackIndex = allTrackIndex(~isAssigned);
            
            % Cells which are labelled as 0 (unassigned) must be clustered
            % and allowed for new object creation (if their dynamic
            % properties allow)
            unassignedCells = currentLabels == 0;
            
            % Cluster unassigned cells to produce new tracks in further
            % steps.
            unassignedGridCells = clusterGridCells(obj, map, unassignedCells);
        end
        
        function updateAssignedTracks(obj, assignedGridCells, assignedTrackIndex)
            % updateAssignedTrack(obj, assignedGridCells, assignedTrackIndex)
            % updates the list of assigned tracks.
            %
            % assignedGridCells(i) is assigned to assignedTrackIndex(i).
            %
            % Track Index is the location index of the track in pTrackList.
            
            % Update function for the track.
            h = obj.TrackUpdateFcn;
            
            % Loop over each assigned track
            for i = 1:numel(assignedTrackIndex)
                % Track to update
                thisTrk = obj.pTrackList(assignedTrackIndex(i));
                
                % Logic to update
                thisTrkLogic = obj.pTrackLogics{assignedTrackIndex(i)};
                
                % Updated track
                thisTrkUpdated = h(thisTrk, assignedGridCells(i));
                
                % Sync properties of track with the updated track. Only a
                % subset of properties can be updated.
                thisTrk = syncTrack(thisTrk, thisTrkUpdated);
                
                % Increment age
                thisTrk.Age = thisTrk.Age + 1;
                
                % Update the track logic of this track
                hit(thisTrkLogic);
                
                thisTrk.TrackLogicState = output(thisTrkLogic);
                
                thisTrk.IsConfirmed = thisTrk.IsConfirmed || thisTrk.ObjectClassID > 0 || checkConfirmation(thisTrkLogic);
                
                % Update the track in the track lists
                obj.pTrackList(assignedTrackIndex(i)) = thisTrk;
                
                % Update the confirmation flags
                obj.pConfirmationFlags(assignedTrackIndex(i)) = thisTrk.IsConfirmed;
            end
        end
        
        function updateUnassignedTracks(obj, unassignedTrackIndex)
            for i = 1:numel(unassignedTrackIndex)
                % Call miss on the track logic
                miss(obj.pTrackLogics{unassignedTrackIndex(i)});
                
                % Set the IsCoasted flag to true
                obj.pTrackList(unassignedTrackIndex(i)).IsCoasted = true;
                
                % Increment age
                obj.pTrackList(unassignedTrackIndex(i)).Age = obj.pTrackList(unassignedTrackIndex(i)).Age + 1;
                
                % Update the track logic state
                obj.pTrackList(unassignedTrackIndex(i)).TrackLogicState = output(obj.pTrackLogics{unassignedTrackIndex(i)});
            end
        end
        
        function clusteredCells = clusterGridCells(obj, map, unassignedGridCells)
            % The cell should be dynamic with a certain degree of
            % confidence
            isDynamicCell = isDynamic(map);
            
            % A dynamic unassigned cell initializes a track
            isInitializing = find(isDynamicCell & unassignedGridCells);
            
            if numel(isInitializing) > 0
                % Get information about all cells
                unassignedCellInfo = getCells(map, isInitializing);
                
                % Cluster unassigned cells
                idx = obj.ClusteringFcn(unassignedCellInfo);
                
                % Get unique values of cluster indices
                uniqueClusters = unique(idx(idx > 0));
                
                % Memory for clusters of grid cells
                clusteredCells = repmat(obj.pSampleCellInformation, numel(uniqueClusters), 1);
                
                % Populate each cluster with required information obtained from
                % the map.
                for i = 1:numel(uniqueClusters)
                    thisClusterIdx = idx == uniqueClusters(i);
                    thisClusterCellIndex = isInitializing(thisClusterIdx);
                    clusteredCells(i) = getCells(map, thisClusterCellIndex);
                end
            else
                % No unassigned cells which are dynamic
                clusteredCells = repmat(obj.pSampleCellInformation,0,1);
            end
        end
        
        function initiateNewTracks(obj, clusteredCells)
            currentNumTracks = obj.pNumLiveTracks;
            numNewTracks = numel(clusteredCells);
            
            for i = 1:numNewTracks
                idx = currentNumTracks + i;
                
                % Initialize the track
                trk = obj.TrackInitializationFcn(clusteredCells(i));
                
                % Initialize the track logic
                init(obj.pTrackLogics{idx});
                
                % Track maitained by the extractor
                internalTrack = obj.pTrackList(idx);
                
                % Update track to get information from new track
                internalTrack = syncTrack(internalTrack, trk);
                
                % Set its age
                internalTrack.Age = uint32(1);
                
                % Set TrackID
                internalTrack.TrackID = getNewTrackID(obj);
                
                % Set the logic state
                internalTrack.TrackLogicState = output(obj.pTrackLogics{idx});
                
                % Track with object class ID gets confirmed immediately.
                internalTrack.IsConfirmed = internalTrack.ObjectClassID > 0 || checkConfirmation(obj.pTrackLogics{idx});
                
                % Update the track in the list
                obj.pTrackList(idx) = internalTrack;
                
                obj.pConfirmationFlags(idx) = internalTrack.IsConfirmed;
            end
            
            % Update the current number
            obj.pNumLiveTracks = obj.pNumLiveTracks + numNewTracks;
        end
        
        function deleteTracks(obj)
            allTracks = obj.pTrackList;
            allLogics = obj.pTrackLogics;
            allFlags = obj.pConfirmationFlags;
            
            toDelete = false(numel(allTracks),1);
            n = numel(allTracks(1).State);
            
            numLive = obj.pNumLiveTracks;
            for i = 1:obj.pNumLiveTracks
                toDelete(i) = checkDeletion(allLogics{i},~allTracks(i).IsConfirmed,allTracks(i).Age);
                if toDelete(i)
                    allTracks(i).State(:) = zeros(n,1);
                    allTracks(i).StateCovariance(:) = eye(n);
                    allTracks(i).IsConfirmed = false;
                    allTracks(i).IsCoasted = false;
                    allTracks(i).ObjectClassID = 0;
                    reset(allLogics{i});
                    allFlags(i) = false;
                    numLive = numLive - 1;
                end
            end
            
            % Set number of live tracks
            obj.pNumLiveTracks = numLive;
            
            % List of tracks to keep
            toKeep = ~toDelete;
                
            % List of tracks and flags to keep
            survivingTracks = allTracks(toKeep);
            survivingFlags = allFlags(toKeep);

            % List of tracks and flags to delete
            otherTracks = allTracks(toDelete);
            otherFlags = allFlags(toDelete);

            % Update the track list
            newTrackList = [survivingTracks;otherTracks];
            obj.pTrackList = newTrackList(1:obj.MaxNumTracks);
            
            % Update the confirmation flag list
            newFlagList = [survivingFlags;otherFlags];
            obj.pConfirmationFlags = newFlagList(1:obj.MaxNumTracks);
            
            % Similarly update the logics, this is a cell array of handle
            % objects and must be handled accordingly.
            if coder.target('MATLAB')
                survivingLogics = allLogics(toKeep);
                otherLogics = allLogics(toDelete);
                obj.pTrackLogics = [survivingLogics;otherLogics];
            else
                toKeepIdx = find(toKeep);
                toDeleteIdx = find(toDelete);
                survivingLogics = cell(numel(toKeepIdx),1);
                otherLogics = cell(numel(toDeleteIdx),1);
                for i = 1:numel(survivingLogics)
                    survivingLogics{i} = obj.pTrackLogics{toKeepIdx(i)};
                end
                for i = 1:numel(otherLogics)
                    otherLogics{i} = obj.pTrackLogics{toDeleteIdx(i)};
                end
                for i = 1:numel(survivingLogics)
                    obj.pTrackLogics{i} = survivingLogics{i};
                end
                for i = 1:numel(otherLogics)
                    obj.pTrackLogics{obj.pNumLiveTracks + i} = otherLogics{i};
                end
            end
            
            
        end
        
        function trID = getNewTrackID(obj)
            % Return new TrackID
            trID = obj.pLastTrackID + 1;
            obj.pLastTrackID = trID;
        end
        
        function [confTracks, tentTracks, allTracks] = extractOutputs(obj)
            % Set StateParameters on the tracks
            for i = 1:obj.pNumLiveTracks
                obj.pTrackList(i).StateParameters = obj.StateParameters;
            end
            % Extract outputs from the tracker
            isConfirmed = obj.pConfirmationFlags(1:obj.pNumLiveTracks);
            
            allTracks = obj.pTrackList(1:obj.pNumLiveTracks);
            confTracks = allTracks(isConfirmed,1);
            tentTracks = allTracks(~isConfirmed,1);
        end
    end
    
    methods (Access = protected)
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            if isLocked(obj)
                s.pTrackList = obj.pTrackList;
                s.pNumLiveTracks = obj.pNumLiveTracks;
                s.pLastTrackID = obj.pLastTrackID;
                s.pConfirmationFlags = obj.pConfirmationFlags;
                s.pLastTime = obj.pLastTime;
                s.pClassToUse = obj.pClassToUse;
                s.pSampleCellInformation = obj.pSampleCellInformation;
                n = numel(obj.pTrackList);
                s.pTrackLogics = cell(n,1);
                for i = 1:n
                    s.pTrackLogics{i} = clone(obj.pTrackLogics{i});
                end
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            if wasLocked
                obj.pTrackList = s.pTrackList;
                obj.pNumLiveTracks = s.pNumLiveTracks;
                obj.pLastTrackID = s.pLastTrackID;
                obj.pConfirmationFlags = s.pConfirmationFlags;
                obj.pLastTime = s.pLastTime;
                obj.pClassToUse = s.pClassToUse;
                obj.pSampleCellInformation = s.pSampleCellInformation;
                obj.pTrackLogics = s.pTrackLogics;
            end
            loadObjectImpl@matlab.System(obj, s, wasLocked)
        end
        
        function newObj = cloneImpl(obj)
            newObj = cloneImpl@matlab.System(obj);
            if coder.internal.is_defined(obj.pTrackList)
                newObj.pTrackList = obj.pTrackList;
                newObj.pNumLiveTracks = obj.pNumLiveTracks;
                newObj.pLastTrackID = obj.pLastTrackID;
                newObj.pConfirmationFlags = obj.pConfirmationFlags;
                newObj.pLastTime = obj.pLastTime;
                newObj.pClassToUse = obj.pClassToUse;
                newObj.pSampleCellInformation = obj.pSampleCellInformation;
                n = numel(obj.pTrackList);
                newObj.pTrackLogics = cell(n,1);
                for i = 1:n
                    newObj.pTrackLogics{i} = clone(obj.pTrackLogics{i});
                end
            end
        end
        
        function resetImpl(obj)
            % Reset extractor to original state
            obj.pLastTime = cast(-eps,'like',obj.pLastTime);
            obj.pConfirmationFlags(:) = false;
            obj.pNumLiveTracks(:) = 0;
            obj.pLastTrackID = uint32(0);
            for i = 1:numel(obj.pTrackLogics)
                reset(obj.pTrackLogics{i});
            end
        end
    end
    
    methods (Static)
        function track = defaultTrackInitialization(gridCells)
            track = fusion.internal.GridMeasurementBuiltins.initRectangularTrack(gridCells);
        end
        
        function track = defaultTrackUpdate(track,gridCells)
            track = fusion.internal.GridMeasurementBuiltins.updateRectangularTrack(track, gridCells);
        end
        function d = defaultTrackDistance(map, track)
            d = fusion.internal.GridMeasurementBuiltins.mapToTrackDistance(map, track);
        end
    end
end

function tr1 = syncTrack(tr1,tr2)
% List of all properties that must be updated from the track to the central
% track.
tr1.State = tr2.State;
tr1.StateCovariance = tr2.StateCovariance;
tr1.ObjectClassID = tr2.ObjectClassID;
tr1.ObjectAttributes = tr2.ObjectAttributes;
end


