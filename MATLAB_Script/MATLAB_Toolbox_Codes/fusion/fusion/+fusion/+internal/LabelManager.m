classdef LabelManager < matlab.System
    % This is an internal class and may be removed or modified in a future release.
    % LabelManager helps the trackerPHD to manage labels on the
    % density.
    % It prunes, unlabel and extract target states and their IDs from the
    % density.
    % It also stores information about which tracks are confirmed.
    
    % Copyright 2018-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Access = {?trackerPHD,?LabelManager,?matlab.unittest.TestCase})
        % A list of current labels
        pCurrentLabels;
        % A flag controlling if track is confirmed
        pConfirmationFlags;
        % Number of labels
        pNumLabels = uint32(0)
    end
    
    properties (Nontunable)
        % Thresholds for labeling. These are passed on to the
        % LabelManagementLogic.
        LabelingThresholds = [1.1 1 0.8];
        % Merging threshold for the density. This is same as trackerPHD.
        MergingThreshold = 25;
        % Extraction threshold is same as trackerPHD.
        ExtractionThreshold = 0.5;
        % Confirmation threshold is same as trackerPHD
        ConfirmationThreshold = 0.8;
        % DeletionThreshold is same as trackerPHD
        DeletionThreshold = 1e-3;
        % Maximum number of labels. Same as MaxNumTracks.
        MaxNumLabels = 1000;
        % TrackerIndex. Same as TrackerIndex in trackerPHD
        TrackerIndex = uint32(0);
    end
    
    properties (Access = {?LabelManager,?matlab.unittest.TestCase})
        % Handles to hold density information for labeled, unlabeled and
        % total density.
        pLabeledDensity
        pUnlabeledDensity
        pTotalDensity;
        
        % Structures which hold track information to be populated.
        pTrackStructs
        
        % A logic object for label management
        pLabelManagementLogic;
        
        pLastLabel = uint32(0);
    end
    properties (Access = {?LabelManager,?matlab.unittest.TestCase},Nontunable)
        % Fields of filter structure
        pFilterFieldNames;
    end
    
    properties (Access = {?trackerPHD,?LabelManager,?matlab.unittest.TestCase},Nontunable)
        pDataType;
    end
    
    methods
        function obj = LabelManager(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,density,stateParams)
            % Allocate memory for densities and state structs
            obj.pLabeledDensity = clone(density);
            obj.pUnlabeledDensity = clone(density);
            obj.pTotalDensity = clone(density);
            nullify(obj.pTotalDensity);
            nullify(obj.pUnlabeledDensity);
            nullify(obj.pLabeledDensity);
            
            if ~coder.internal.is_defined(obj.pDataType)
            	obj.pDataType = class(obj.pTotalDensity.Weights);
            end
            
            % defaultTrackStruct
            [totalStruct, filterFields] = fusion.internal.LabelManager.defaultOutput(density, stateParams);
            obj.pFilterFieldNames = filterFields;
            
            % Add TrackerIndex
            totalStruct.SourceIndex(:) = obj.TrackerIndex;  % Preserve data type to uint32
            
            % Allocate memory for structs
            obj.pTrackStructs = repmat(totalStruct,[obj.MaxNumLabels 1]);
            
            % Allocate memory for labels
            obj.pCurrentLabels = zeros(obj.MaxNumLabels,1,'uint32');
            
            % Allocate memory for confirmation flags
            obj.pConfirmationFlags = false(obj.MaxNumLabels,1);
            
            % Assign label management logic
            obj.pLabelManagementLogic = fusion.internal.LabelManagementLogic(obj.LabelingThresholds);
        end
        
        function [confTracks,tentTracks,allTracks,extractionAnalysis] = stepImpl(obj,density,stateParams,time)
            % Prune density first
            toPrune = density.Weights < obj.DeletionThreshold;
            prune(density,toPrune)
                    
            % All tracks are at the same time and report the same state
            % parameters
            for i = 1:obj.MaxNumLabels
                obj.pTrackStructs(i).UpdateTime = time;
                obj.pTrackStructs(i).StateParameters = stateParams;
            end
            
            % Sync densities
            sync(obj.pLabeledDensity,density);
            sync(obj.pUnlabeledDensity,density);
            sync(obj.pTotalDensity,density);
            U = obj.MergingThreshold;
            
            % Nullify total density
            nullify(obj.pTotalDensity);
            labeledDensity(obj.pUnlabeledDensity,uint32(0));
            
            % Current labels
            numCurrentLabels = obj.pNumLabels;
            currentLabels = obj.pCurrentLabels(1:numCurrentLabels);
            tIDBegin = currentLabels;
            isConfirmed = obj.pConfirmationFlags(1:numCurrentLabels);
            isDeleted = false(numCurrentLabels,1);
            
            % Now go through the list of current labels
            for i = 1:numCurrentLabels
                thisLabel = currentLabels(i);
                sync(obj.pLabeledDensity,density);
                labeledDensity(obj.pLabeledDensity,thisLabel);
                merge(obj.pLabeledDensity,U);
                if obj.pLabeledDensity.NumComponents == 0
                    isDeleted(i) = true;
                    toPrune = false;
                    toUnlabel = false;
                else
                    [toPrune, toUnlabel] = manageHypothesis(obj.pLabelManagementLogic,obj.pLabeledDensity);
                end
                if any(toPrune)
                    [thisTrackState, isDeleted(i),isConfirmed(i)] = pruneAndExtract(obj,toPrune,isConfirmed(i));
                elseif any(toUnlabel)
                    [thisTrackState, isDeleted(i),isConfirmed(i)] = unlabelAndExtract(obj,toUnlabel,thisLabel,isConfirmed(i));
                else
                    [thisTrackState, isDeleted(i),isConfirmed(i)] = simplyExtract(obj,isConfirmed(i));
                end
                if ~isDeleted(i)
                    trackStruct = syncStructs(obj,obj.pTrackStructs(i),thisTrackState);
                    obj.pTrackStructs(i) = trackStruct;
                    obj.pTrackStructs(i).Age = obj.pTrackStructs(i).Age + 1;
                    obj.pTrackStructs(i).IsConfirmed = isConfirmed(i);
                end
            end
            tIDDeleted = tIDBegin(isDeleted);
            
            % Now go through unlabeled density
            unlabeledDensity = obj.pUnlabeledDensity;
            append(unlabeledDensity,obj.pTotalDensity);
            % Get unlabled density using 0 label.
            labeledDensity(unlabeledDensity,uint32(0));
            
            % Merge unlabeled density
            merge(unlabeledDensity,U);
            
            % Label the unlabeled density
            [newStates,indices] = extractState(obj.pUnlabeledDensity,obj.ExtractionThreshold);
            
            numNewStates = numel(newStates);
            newIsConfirmed = false(numNewStates,1);
            
            for i = 1:size(indices,2)
                thisTrackState = newStates(i);
                thisIndex = numCurrentLabels + i;
                newID = getNewLabel(obj);
                if newID == 0 % MaxNumTracks reached
                    numNewStates = i-1;
                    newIsConfirmed = newIsConfirmed(1:(i-1));
                    break;
                end
                trackStruct = syncStructs(obj,obj.pTrackStructs(thisIndex),thisTrackState);
                obj.pTrackStructs(thisIndex) = trackStruct;
                obj.pTrackStructs(thisIndex).Age = 0;
                obj.pTrackStructs(thisIndex).TrackID = newID;
                obj.pCurrentLabels(thisIndex) = newID;
                unlabeledDensity.Labels(indices(:,i)) = newID;
                newIsConfirmed(i) = sum(unlabeledDensity.Weights(indices(:,i))) > obj.ConfirmationThreshold;
                obj.pTrackStructs(thisIndex).IsConfirmed = newIsConfirmed(i);
            end
            
            % Update the input density.
            append(obj.pTotalDensity,unlabeledDensity);
            sync(density,obj.pTotalDensity);
            
            totalIsConfirmed = [isConfirmed;newIsConfirmed];
            totalIsDeleted =   [isDeleted;false(numNewStates,1)];
            allTracksWithDeleted = obj.pTrackStructs(1:(numCurrentLabels + numNewStates));
            confTracks = allTracksWithDeleted(totalIsConfirmed & ~totalIsDeleted);
            tentTracks = allTracksWithDeleted(~totalIsConfirmed & ~totalIsDeleted);
            allTracks = allTracksWithDeleted(~totalIsDeleted);
            obj.pNumLabels(1) = numel(allTracks);
            if coder.target('MATLAB')
                obj.pCurrentLabels(1:obj.pNumLabels) = [allTracks.TrackID];
                obj.pConfirmationFlags(1:obj.pNumLabels) = [allTracks.IsConfirmed];
            else
                for i = 1:numel(allTracks)
                    obj.pCurrentLabels(i) = allTracks(i).TrackID;
                    obj.pConfirmationFlags(i) = allTracks(i).IsConfirmed;
                end
            end
            obj.pTrackStructs(1:obj.pNumLabels) = allTracks;
            % Assemble extraction analysis
            if nargout == 4
                extractionAnalysis = struct('TrackIDsAtStepBeginning',tIDBegin,...
                    'DeletedTracksID',tIDDeleted,...
                    'TrackIDsAtStepEnd',obj.pCurrentLabels(1:obj.pNumLabels)...
                    );
                
            end
        end
        
        function resetImpl(obj)
            % Initialize/ rest the LabelManager
            
            nullify(obj.pTotalDensity);
            nullify(obj.pUnlabeledDensity);
            nullify(obj.pLabeledDensity);
            obj.pCurrentLabels(:) = 0;
            obj.pConfirmationFlags(:) = false;
            obj.pLastLabel = uint32(0);
            obj.pNumLabels = uint32(0);
        end
        
        function releaseImpl(obj)
            % Release resource
            obj.pNumLabels = uint32(0);
            obj.pCurrentLabels(:) = 0;
            obj.pLastLabel = uint32(0);
            obj.pConfirmationFlags(:) = false;
        end
        
        function s = saveObjectImpl(obj)
            % Save properties in a struct s
            % Save base class
            s = saveObjectImpl@matlab.System(obj);
            s.LabelingThresholds = obj.LabelingThresholds;
            s.MergingThreshold = obj.MergingThreshold;
            s.ExtractionThreshold = obj.ExtractionThreshold;
            s.DeletionThreshold = obj.DeletionThreshold;
            s.MaxNumLabels = obj.MaxNumLabels;
            if isLocked(obj)
                s.pCurrentLabels = obj.pCurrentLabels;
                s.pConfirmationFlags = obj.pConfirmationFlags;
                s.pNumLabels = obj.pNumLabels;
                s.pLabeledDensity = obj.pLabeledDensity;
                s.pUnlabeledDensity = obj.pUnlabeledDensity;
                s.pTotalDensity = obj.pTotalDensity;
                s.pTrackStructs = obj.pTrackStructs;
                s.pLabelManagementLogic = obj.pLabelManagementLogic;
                s.pFilterFieldNames = obj.pFilterFieldNames;
                s.pDataType = obj.pDataType;
                s.pLastLabel = obj.pLastLabel;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            obj.LabelingThresholds = s.LabelingThresholds;
            obj.MergingThreshold = s.MergingThreshold;
            obj.ExtractionThreshold = s.ExtractionThreshold;
            obj.DeletionThreshold = s.DeletionThreshold;
            obj.MaxNumLabels = s.MaxNumLabels;
            if wasLocked
                obj.pCurrentLabels = s.pCurrentLabels;
                obj.pConfirmationFlags = s.pConfirmationFlags;
                obj.pNumLabels = s.pNumLabels;
                obj.pLabeledDensity = s.pLabeledDensity;
                obj.pUnlabeledDensity = s.pUnlabeledDensity;
                obj.pTotalDensity = s.pTotalDensity;
                obj.pTrackStructs = s.pTrackStructs;
                obj.pLabelManagementLogic = s.pLabelManagementLogic;
                obj.pDataType = s.pDataType;
                obj.pFilterFieldNames = s.pFilterFieldNames;
                if isfield(s,'pLastLabel')
                    obj.pLastLabel = s.pLastLabel;
                else
                    obj.pLastLabel = max(s.pCurrentLabels);
                end
            end
            % Load base class
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    methods (Access = {?LabelManager,?matlab.unittest.TestCase})
        function newLabel = getNewLabel(obj)
            % Generate a new label
            newLabel = obj.pLastLabel + 1;
            if obj.pNumLabels + 1 > obj.MaxNumLabels
                coder.internal.warning('fusion:trackerPHD:MaxNumTracksReached','MaxNumTracks');
                newLabel(:) = 0;
                return;
            end
            obj.pNumLabels = obj.pNumLabels + 1;
            obj.pCurrentLabels(obj.pNumLabels) = newLabel;
            obj.pLastLabel = newLabel;
        end
        
        function [state,isDeleted,isConfirmed] = pruneAndExtract(obj,toPrune,isConfirmed)
            prune(obj.pLabeledDensity,toPrune)
            % Extract the state using 0 threshold.
            state = extractState(obj.pLabeledDensity,cast(0,obj.pDataType));
            isDeleted = obj.pLabeledDensity.NumComponents == 0;
            % Add this density to total density
            append(obj.pTotalDensity,obj.pLabeledDensity);
            isConfirmed = isConfirmed || sum(obj.pLabeledDensity.Weights) > obj.ConfirmationThreshold;
        end
        
        function [state,isDeleted,isConfirmed] = unlabelAndExtract(obj,toUnlabel,thisLabel,isConfirmed)
            thisDensity = obj.pLabeledDensity;
            thisDensity.Labels(toUnlabel) = uint32(0);
            append(obj.pTotalDensity,thisDensity);
            % From thisDensity, extract the labeled component
            labeledDensity(thisDensity,thisLabel);
            state = extractState(thisDensity,cast(0,obj.pDataType));
            isDeleted = thisDensity.NumComponents == 0;
            isConfirmed = isConfirmed || sum(thisDensity.Weights) > obj.ConfirmationThreshold;
        end
        
        function [state,isDeleted,isConfirmed] = simplyExtract(obj,isConfirmed)
            thisDensity = obj.pLabeledDensity;
            isDeleted = thisDensity.NumComponents == 0;
            state = extractState(thisDensity,cast(0,obj.pDataType));
            isConfirmed = isConfirmed || sum(thisDensity.Weights) > obj.ConfirmationThreshold;
            append(obj.pTotalDensity,thisDensity);
        end
    end
    
    methods (Hidden)
        % Avoid setting up the object to output tracks when the density has
        % no components        
        function trackStruct = syncStructs(obj,trackStruct,filterStruct)
            filterFieldNames = obj.pFilterFieldNames;
            for i = 1:numel(filterFieldNames)
                trackStruct.(filterFieldNames{i}) = filterStruct(1).(filterFieldNames{i});
            end
        end
        
        function predictedTracks = predictTracksToTime(obj,density,id,dT,notStateOnly)
            coder.internal.assert(isLocked(obj),'fusion:trackerPHD:PredictBeforeUpdate');
            currentLabels = obj.pCurrentLabels(1:obj.pNumLabels);
            isConfirmed = obj.pConfirmationFlags(1:obj.pNumLabels);
            currentTracks = obj.pTrackStructs(1:obj.pNumLabels);
            switch id
                case -1
                    predictedTracks = currentTracks;
                    labels = currentLabels;
                case -2
                    predictedTracks = currentTracks(isConfirmed);
                    labels = currentLabels(isConfirmed);
                case -3
                    predictedTracks = currentTracks(~isConfirmed);
                    labels = currentLabels(~isConfirmed);
                otherwise
                    trackIndex = currentLabels == id;
                    coder.internal.assert(any(trackIndex),'fusion:trackerPHD:TrackUnavailable',id(1));
                    predictedTracks = currentTracks(trackIndex);
                    labels = currentLabels(trackIndex);
            end
            if ~isempty(predictedTracks)
                if ~notStateOnly
                    predictedTracks = predictTracksStateOnly(obj,predictedTracks,density,dT);
                else
                    predictedTracks = predictTracksFull(obj,predictedTracks,density,labels,dT);
                end
            end
        end
        
        function predictedTracks = predictTracksFull(obj,predictedTracks,density,labels,dT)
            % Use the density to predict the track. Do not change the state of the density.
            % We can use the pLabeledDensity here to do the predicition
            for i = 1:numel(labels)
                sync(obj.pLabeledDensity,density);
                labeledDensity(obj.pLabeledDensity,labels(i));
                predict(obj.pLabeledDensity,dT);
                filterState = extractState(obj.pLabeledDensity,0);
                coder.internal.assert(~isempty(filterState),'fusion:trackerPHD:PreviouslyAvailableLost');
                predictedTracks(i) = syncStructs(obj,predictedTracks(i),filterState(1));
                predictedTracks(i).UpdateTime = predictedTracks(i).UpdateTime + dT;
            end
            nullify(obj.pLabeledDensity);
        end
        
        function predictedTracks = predictTracksStateOnly(~,predictedTracks,density,dT)
            if coder.target('MATLAB')
                % It cannot be row, because PHD filter uses columns.
                x = cat(2,predictedTracks.State);
            else
                n = numel(predictedTracks);
                p = numel(predictedTracks(1).State);
                x = zeros(p,n,'like',predictedTracks(1).State);
                for i = 1:n
                    x(:,i) = predictedTracks(i).State;
                end
            end
            f = models(density);
            xT = f(x,dT);
            for i = 1:numel(predictedTracks)
                predictedTracks(i).State = xT(:,i);
                predictedTracks(i).UpdateTime = predictedTracks(i).UpdateTime + dT;
            end
        end
        
        function trID = initializeTrack(obj, totalPHD, track, newTrackPHD)
            % initializeTrack(obj, totalPHD, track) initializes 1 component
            % in the totalPHD which reflects the state of the input track.
            % You can use this signature with Gaussian filter, gmphd.
            % 
            % initializeTrack(obj, totalPHD, track, newTrackPHD)
            % initializes new components in the totalPHD based on the
            % components provided by newTrackPHD. All these components
            % belong to the same track. You can use this signature with
            % Gaussian and Non-Gaussian filters such as gmphd and ggiwphd.
            %
            if nargin == 3
                % If PHD filter is not provided, get the "best" possible
                % filter by asking the estimated filter.
                %
                % If a phd filter cannot initialize itself from a track,
                % for example, ggiwphd, it should error out. The error must
                % be implemented by the filter to avoid tracker carrying
                % information if a filter is Gaussian or not.
                newTrackPHD = totalPHD.initializeFromTrack(track);
            else % If components are provided
                % Assert that class of input filter is same as class of 
                % filter maintained by the tracker
                coder.internal.assert(isa(newTrackPHD,class(totalPHD)),'fusion:trackerPHD:initializeMismatchPHD'); 
                
                % Also assert that it has atleast one component
                coder.internal.assert(newTrackPHD.NumComponents > 0, 'fusion:trackerPHD:initializeNoComponents');
            end
            
            % Assign the PHD filter a correct label (new track ID)
            trID = getNewLabel(obj); % This increments the number of labels
            
            if trID == 0 % MaxNumTracks reached, don't do anything
                return;
            end
            newTrackPHD.Labels(:) = trID;
                      
            % If and only if track TrackLogicState set to Integrated, the
            % weights of the filter will be modified by considering it as
            % "probability of existence". Otherwise, the track will be
            % initialized considering existence probability as 1. 
            % Note: Sum of weights of components is equal to number of
            % targets they represent.
            if strcmpi(track.TrackLogic,'integrated')
                Pte = cast(track.TrackLogicState,obj.pDataType);
            else
                Pte = cast(1,obj.pDataType);
            end
            scale(newTrackPHD,Pte/sum(newTrackPHD.Weights));

            % Append this to current phd filter maintained by tracker
            append(totalPHD, newTrackPHD);
            
            % Update the track struct with required information from the
            % input track
            updateTrackStruct(obj, obj.pNumLabels, track, trID);
        end
        
        function updateTrackStruct(obj, idx, track, trID)
            % Update the information of the maintained track with an input
            % track. 
            thisTrackStruct = obj.pTrackStructs(idx);
            
            % Subscripted assignment to maintain data types.
            thisTrackStruct.TrackID(1) = trID; 
            thisTrackStruct.Age(1) = track.Age;
            thisTrackStruct.IsConfirmed(1) = track.IsConfirmed;
            thisTrackStruct.UpdateTime(1) = track.UpdateTime;
            
            % Assign it back to the track list
            obj.pTrackStructs(idx) = thisTrackStruct;
            
            % Set confirmation flag
            obj.pConfirmationFlags(idx) = track.IsConfirmed;
        end
        
        function deleted = deleteTrack(obj, currentPHD, trkID)
            % First check if the track exists
            currentNumLabels = obj.pNumLabels;
            currentLabels = obj.pCurrentLabels(1:currentNumLabels);
            currentConfFlags = obj.pConfirmationFlags(1:currentNumLabels);
            currentTrackList = obj.pTrackStructs(1:currentNumLabels);
            
            isSameLabel = currentLabels == trkID;
            
            % If Track does not exist, throw a warning and return deleted
            % as false.
            if ~any(isSameLabel)
                deleted = false;
                coder.internal.warning('fusion:trackerPHD:TrackUnavailable',trkID);
                return;
            end
            
            % If Track Exists
            % 1. Make LabelManager forget that its a maintained track
            obj.pNumLabels = obj.pNumLabels - 1;
            obj.pCurrentLabels(1:obj.pNumLabels) = currentLabels(~isSameLabel);
            obj.pConfirmationFlags(1:obj.pNumLabels) = currentConfFlags(~isSameLabel);
            obj.pTrackStructs(1:obj.pNumLabels) = currentTrackList(~isSameLabel);
            
            % 2. Delete components of this TrackID from the filter
            componentWithSameLabel = currentPHD.Labels == trkID;
            % Use prune to delete them
            prune(currentPHD, componentWithSameLabel);
            deleted = true;
        end
    end
    
    methods (Static)
        function trackStructUpper = getDefaultTrackStructUpper(classToUse)
            trackStructUpper = struct('TrackID',uint32(0),...
                'SourceIndex',uint32(0),...
                'UpdateTime',cast(0,classToUse),...
                'Age',cast(0,classToUse));
        end
        
        function trackStructLower = getDefaultTrackStructLower(stateParams,classToUse)
           % Get lower portion of track struct. In the middle are appended
           % information reported by the filter.
           trackStructLower = struct('StateParameters',stateParams,...
               'ObjectClassID',cast(0,classToUse),...
               'IsConfirmed',false,...
               'IsCoasted',false,...
               'IsSelfReported',true); % Always self reported
        end
        
        function [totalStruct, filterFields] = defaultOutput(phd, stateParams)
            % Every PHD filter must carry weights. It is an abstract
            % property.
            classToUse = class(phd.Weights);
            defaultTrackStructUpper = fusion.internal.LabelManager.getDefaultTrackStructUpper(classToUse);
            defaultTrackStructLower = fusion.internal.LabelManager.getDefaultTrackStructLower(stateParams, classToUse);
            
            % filterTrackStruct. This will also have State and StateCovariance
            filterTrackStruct = sampleStruct(phd);
            filterFields = fieldnames(filterTrackStruct);
            
            % Start with upper structs
            totalStruct = defaultTrackStructUpper;
            
            % Add filter struct below it
            fields = fieldnames(filterTrackStruct);
            for i = 1:numel(fields)
                totalStruct.(fields{i}) = filterTrackStruct.(fields{i});
            end
            
            % Add lower struct below it
            fields = fieldnames(defaultTrackStructLower);
            for i = 1:numel(fields)
                totalStruct.(fields{i}) = defaultTrackStructLower.(fields{i});
            end
        end
        
    end
end