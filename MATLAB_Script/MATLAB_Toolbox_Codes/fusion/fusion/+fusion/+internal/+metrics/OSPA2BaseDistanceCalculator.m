classdef OSPA2BaseDistanceCalculator < matlab.System
    % This is an internal class and may be removed or modified in a future
    % release. 

    % Copyright 2021 The MathWorks, Inc.

    % This class calculates the OSPA base distance between history of
    % tracks and history of truths using a weighted-sum in the last N
    % steps, where N is defined using WindowLength.

    %#codegen
    %#internal

    properties
        CutoffDistance = 30
        WindowSumOrder = 2
        Weights
    end

    properties (Nontunable)
        WindowLength = 100
        TrackIdentifierFcn
        TruthIdentifierFcn
        DistanceFcn
    end

    properties (Access = {?fusion.internal.metrics.OSPA2BaseDistanceCalculator,...
            ?fusion.internal.metrics.OSPABase,...
            ?matlab.unittest.TestCase})
        CurrentStep = uint32(0);
        TrackIDs
        TruthIDs
        TrackExistence
        TruthExistence
        HistoryDistanceMatrix
    end

    properties (Access = {?fusion.internal.metrics.OSPA2BaseDistanceCalculator,...
            ?fusion.internal.metrics.OSPABase,...
            ?matlab.unittest.TestCase}, Dependent)
        WindowIndex
        CurrentWeights
        CurrentTrackExistence
        CurrentTruthExistence
        CurrentDistance
    end

    % Get methods to obtain values in the valid window index, when time
    % step may be less than the window length
    methods
        function idx = get.WindowIndex(obj)
            startIdx = max(1,obj.WindowLength - obj.CurrentStep + 1);
            endIdx = obj.WindowLength;
            idx = startIdx:endIdx;
        end

        function w = get.CurrentWeights(obj)
            w = obj.Weights(1,obj.WindowIndex);
            w = w/sum(w);
        end

        function tf = get.CurrentTrackExistence(obj)
            tf = obj.TrackExistence(:,obj.WindowIndex);
        end

        function tf = get.CurrentTruthExistence(obj)
            tf = obj.TruthExistence(:,obj.WindowIndex);
        end

        function d = get.CurrentDistance(obj)
            d = obj.HistoryDistanceMatrix(:,:,obj.WindowIndex);
        end
    end

    methods
        function obj = OSPA2BaseDistanceCalculator(varargin)
            setProperties (obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, varargin)
            initializeInternalProperties(obj, varargin{:});
        end

        % Compute distance matrix as step
        function dMatrix = stepImpl(obj, tracks, truths)
            % Get IDs of tracks and truths at current step
            trackIDs = obj.TrackIdentifierFcn(tracks);
            truthIDs = obj.TruthIdentifierFcn(truths);

            % Update history to accomodate new tracks and truths
            newTrackIDs = setdiff(trackIDs, obj.TrackIDs, 'stable');
            if ~isempty(newTrackIDs)
                insertTrack(obj, newTrackIDs);
            end
            newTruthIDs = setdiff(truthIDs, obj.TruthIDs, 'stable');
            if ~isempty(newTruthIDs)
                insertTruth(obj, newTruthIDs);
            end
            
            % Update track existence
            isAvailable = ismember(obj.TrackIDs, trackIDs);
            trackExistence = [obj.TrackExistence(:,2:end) isAvailable(:,1)];
            obj.TrackExistence = trackExistence(:,1:obj.WindowLength);

            % Update truth existence
            isAvailable = ismember(obj.TruthIDs, truthIDs);
            truthExistence = [obj.TruthExistence(:,2:end) isAvailable];
            obj.TruthExistence = truthExistence(:,1:obj.WindowLength);

            % Update step
            obj.CurrentStep = obj.CurrentStep + 1;

            % Remove old tracks
            tracksNotExist = all(~obj.TrackExistence,2);
            obj.TrackIDs = obj.TrackIDs(~tracksNotExist,1);
            obj.TrackExistence = obj.TrackExistence(~tracksNotExist,:);
            obj.HistoryDistanceMatrix = obj.HistoryDistanceMatrix(~tracksNotExist,:,:);

            % Remove old truths
            truthsNotExist = all(~obj.TruthExistence,2);
            obj.TruthIDs = obj.TruthIDs(~truthsNotExist,1);
            obj.TruthExistence = obj.TruthExistence(~truthsNotExist,:);
            obj.HistoryDistanceMatrix = obj.HistoryDistanceMatrix(:,~truthsNotExist,:);

            % Calculate distance matrix
            trackExist = obj.TrackExistence(:,end);
            truthExist = obj.TruthExistence(:,end);
            d = cat(3,obj.HistoryDistanceMatrix(:,:,2:end), ...
                zeros(size(obj.HistoryDistanceMatrix,1),size(obj.HistoryDistanceMatrix,2),'like',obj.HistoryDistanceMatrix));

            % Distance for last step
            dStep = d(:,:,end);

            % Go over each existing track in history
            for i = 1:numel(obj.TrackIDs)
                % Go over each existing truth in history
                for j = 1:numel(obj.TruthIDs)
                    % If both exist at this step
                    if trackExist(i) && truthExist(j)
                        % Find the track and truth
                        if coder.target('MATLAB')
                            thisTrack = tracks(trackIDs == obj.TrackIDs(i));
                            thisTruth = truths(truthIDs == obj.TruthIDs(j));
                        else
                            trkIdx = find(trackIDs == obj.TrackIDs(i),1,'first');
                            truthIdx = find(truthIDs == obj.TruthIDs(j),1,'first');
                            thisTrack = tracks(trkIdx(1));
                            thisTruth = truths(truthIdx(1));
                        end
                        dStep(i,j) = min(obj.CutoffDistance, obj.DistanceFcn(thisTrack, thisTruth));
                    % One exists at this step
                    elseif trackExist(i) || truthExist(j)
                        dStep(i,j) = obj.CutoffDistance;
                    else % None exists at this step
                        dStep(i,j) = 0;
                    end
                end
            end

            % Set the DistanceMatrix to store history
            d(:,:,end) = dStep;
            obj.HistoryDistanceMatrix = d;

            % Weighted qth order sum in the window
            w = reshape(obj.CurrentWeights,1,1,[]);
            d = obj.CurrentDistance;
            q = obj.WindowSumOrder;
            dMatrix = (sum(w.*(d.^q),3)).^(1/q);
        end

        % insertTrack inserts a new track into the history. When a new
        % track appears, we need to add its distance in the entire window.
        % It's distance against truths is defined as c when truth existed
        % and 0 when truth didn't exist
        function insertTrack(obj, newTrackIDs)
            d = obj.HistoryDistanceMatrix;
            numNewTracks = numel(newTrackIDs);
            exist = obj.CurrentTruthExistence;
            c = obj.CutoffDistance;
            dNewTracksExistingTruth = exist*c;
            numTruths = size(d,2);
            wl = obj.WindowLength;
            for i = 1:numNewTracks
                d = cat(1,d,zeros(1,numTruths,wl,'like',d));
                d(end,:,obj.WindowIndex) = dNewTracksExistingTruth;
            end
            obj.HistoryDistanceMatrix = d;
            obj.TrackIDs = [obj.TrackIDs;newTrackIDs];
            obj.TrackExistence = [obj.TrackExistence;false(numNewTracks,obj.WindowLength)];
        end

        % insertTruth inserts a new truth into the history. When a new
        % truth appears, we need to add its distance in the entire window.
        % It's distance against tracks is defined as c when track existed
        % and 0 when track didn't exist
        function insertTruth(obj, newTruthIDs)
            d = obj.HistoryDistanceMatrix;
            numNewTruths = numel(newTruthIDs);
            exist = obj.CurrentTrackExistence;
            c = obj.CutoffDistance;
            dNewTruthsExistingTracks = exist*c;
            numTracks = size(d,1);
            wl = size(d,3);
            for i = 1:numNewTruths
                d = cat(2,d,zeros(numTracks,1,wl,'like',d));
                d(:,end,obj.WindowIndex) = dNewTruthsExistingTracks;
            end
            obj.TruthIDs = [obj.TruthIDs;newTruthIDs];
            obj.TruthExistence = [obj.TruthExistence;false(numNewTruths,obj.WindowLength)];
            obj.HistoryDistanceMatrix = d;
        end

        function initializeInternalProperties(obj, varargin)
            % Set variable-sized trackIDs and truthIDs
            trackIDs = zeros(0,1,'uint32'); % NumTracks x 1
            coder.varsize('trackIDs',[inf 1],[1 0]);
            truthIDs = zeros(0,1,'uint32'); % NumTruths x 1
            coder.varsize('truthIDs',[inf 1],[1 0]);
            obj.TrackIDs = trackIDs;
            obj.TruthIDs = truthIDs;            
            
            % Set variable-sized existence for tracks and truths
            trackExistence = false(0,obj.WindowLength); % NumTracks x NumSteps
            coder.varsize('trackExistence',[inf obj.WindowLength],[1 0]);
            truthExistence = false(0,obj.WindowLength); % NumTruths x NumSteps
            coder.varsize('truthExistence',[inf obj.WindowLength],[1 0]);
            obj.TrackExistence = trackExistence;
            obj.TruthExistence = truthExistence;
            
            % Variable-sized distance matrix
            d = zeros(0,0,obj.WindowLength,'like',obj.CutoffDistance);
            coder.varsize('d',[inf inf obj.WindowLength],[1 1 0]);
            obj.HistoryDistanceMatrix = d;
        end

    end

    %% release, reset, save and load methods
    methods (Access = protected)
        function releaseImpl(obj)
            obj.CurrentStep = uint32(0);
        end

        function resetImpl(obj)
            obj.CurrentStep = uint32(0);
            initializeInternalProperties(obj);
        end

        function loadObjectImpl(obj, s, wasLocked)
            if wasLocked
                props = obj.protectedPropertiesToSaveLoad;
                for i = 1:numel(props)
                    obj.(props{i}) = s.(props{i});
                end
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            if isLocked(obj)
                props = obj.protectedPropertiesToSaveLoad;
                for i = 1:numel(props)
                    s.(props{i}) = obj.(props{i});
                end
            end
        end
    end

    methods (Access = protected, Static)
        function props = protectedPropertiesToSaveLoad
            props = {'CurrentStep',...
                'TrackIDs',...
                'TruthIDs',...
                'TrackExistence',...
                'TruthExistence',...
                'HistoryDistanceMatrix'};
        end
    end
end

