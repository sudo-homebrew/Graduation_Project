classdef AssignmentMetrics < handle%
%

%   Copyright 2018 The MathWorks, Inc.

    properties
        AssignmentThreshold
        DivergenceThreshold
        
        DistanceFunctionFormat
        
        MotionModel
        AssignmentDistance
        DivergenceDistance
        
        AssignmentDistanceFcn
        DivergenceDistanceFcn
        IsInsideCoverageAreaFcn
        
        InvalidTrackID
        InvalidTruthID
    end
    
    properties (Dependent)
        TruthIdentifierFcn
        TrackIdentifierFcn
    end
        
    properties  (Access = {?matlab.unittest.TestCase})
        Tracks
        Truths
        TrackHistory
        TruthHistory
    end

    properties (Constant, Access = private)
        %             model      distance      distanceFcn  
        Builtins = {'constvel',  'posnees',    @neesposcv
                    'constvel',  'velnees',    @neesvelcv
                    'constvel',  'posabserr',  @abserrposcv
                    'constvel',  'velabserr',  @abserrvelcv
                    'constacc',  'posnees',    @neesposca
                    'constacc',  'velnees',    @neesvelca
                    'constacc',  'posabserr',  @abserrposca
                    'constacc',  'velabserr',  @abserrvelca
                    'constturn', 'posnees',    @neesposct
                    'constturn', 'velnees',    @neesvelct
                    'constturn', 'posabserr',  @abserrposct
                    'constturn', 'velabserr',  @abserrvelct
                    'singer',    'posnees',    @neesposca
                    'singer',    'velnees',    @neesvelca
                    'singer',    'posabserr',  @abserrposca
                    'singer',    'velabserr',  @abserrvelca}
    end
    
    methods
        
        function value = get.TruthIdentifierFcn(obj)
            value = obj.TruthHistory.IdentifierFcn;
        end
        
        function value = get.TrackIdentifierFcn(obj)
            value = obj.TrackHistory.IdentifierFcn;
        end
                
        function set.TruthIdentifierFcn(obj, value)
            obj.TruthHistory.IdentifierFcn = value;
        end
        
        function set.TrackIdentifierFcn(obj, value)
            obj.TrackHistory.IdentifierFcn = value;
        end
         
        function obj = AssignmentMetrics
            obj.AssignmentThreshold = 1;
            obj.DivergenceThreshold = 2;
            
            obj.DistanceFunctionFormat = 'built-in';
            obj.MotionModel = 'constvel';
            
            obj.AssignmentDistance = 'posnees';
            obj.AssignmentDistanceFcn = @neesposcv;
            
            obj.DivergenceDistance = 'posnees';
            obj.DivergenceDistanceFcn = @neesposcv;
            
            obj.InvalidTrackID = NaN;
            obj.InvalidTruthID = NaN;
            
            obj.TrackHistory = fusion.internal.metrics.TrackHistory(obj.InvalidTrackID, obj.InvalidTruthID);
            obj.TrackIdentifierFcn = @defaultTrackIdentifier;
            
            obj.TruthHistory = fusion.internal.metrics.TruthHistory(obj.InvalidTrackID, obj.InvalidTruthID);
            obj.TruthIdentifierFcn = @defaultTruthIdentifier;
            
            obj.IsInsideCoverageAreaFcn = @defaultIsInsideCoverageArea;
            
        end
        
        function reset(obj)
            reset(obj.TrackHistory, obj.InvalidTrackID, obj.InvalidTruthID);
            reset(obj.TruthHistory, obj.InvalidTrackID, obj.InvalidTruthID);
        end
        
        function analyze(obj, tracks, truths)
            [iDeletedTrack, iExistingTrack, iNewTrack] = initTracks(obj, tracks);
            [iDeletedTruth, iExistingTruth, iNewTruth] = initTruths(obj, truths); %#ok<ASGLU>
            
            analyzeDeletedTracks(obj, iDeletedTrack);
            analyzeExistingTracks(obj, iExistingTrack, iExistingTruth, iDeletedTruth);
            analyzeNewTracks(obj, iNewTrack);
            
            age(obj.TruthHistory);
            age(obj.TrackHistory);
        end
        
        function [trackIDs, truthIDs] = currentAssignment(obj)
            current = obj.TrackHistory.Current;
            iAssigned = find(~current.FalseStatus);
            trackIDs = current.TrackID(iAssigned);
            truthIDs = current.TruthID(iAssigned);
        end
        
        function tms = trackMetricsSummary(obj)
            tms = summary(obj.TrackHistory);
        end
        
        function tms = truthMetricsSummary(obj)
            tms = summary(obj.TruthHistory);
        end
        
        function tmt = trackMetricsTable(obj)
            tmt = metricsTable(obj.TrackHistory);
        end
        
        function tmt = truthMetricsTable(obj)
            tmt = metricsTable(obj.TruthHistory);
        end
        
        function setBuiltinDistanceFunctions(obj)
            rm = strcmp(obj.MotionModel, obj.Builtins(:,1));
            ra = strcmp(obj.AssignmentDistance, obj.Builtins(:,2));
            rd = strcmp(obj.DivergenceDistance, obj.Builtins(:,2));
            obj.AssignmentDistanceFcn = obj.Builtins{rm & ra, 3};
            obj.DivergenceDistanceFcn = obj.Builtins{rm & rd, 3};
        end
        
        function s = saveHistory(obj)
            s.TrackHistory = saveHistory(obj.TrackHistory);
            s.TruthHistory = saveHistory(obj.TruthHistory);
        end
        
        function loadHistory(obj, s)
            loadHistory(obj.TrackHistory, s.TrackHistory);
            loadHistory(obj.TruthHistory, s.TruthHistory);
        end
        
    end
    
    methods (Access = private)
        
        function [iDeletedTrack, iExistingTrack, iNewTrack] = initTracks(obj, tracks)
            [sortIdx, iDeletedTrack, iExistingTrack, iNewTrack] = init(obj.TrackHistory,tracks);
            obj.Tracks = reshape(tracks(sortIdx),[],1);
        end
        
        function [iDeletedTruth, iExistingTruth, iNewTruth] = initTruths(obj, truths)
            [sortIdx, iDeletedTruth, iExistingTruth, iNewTruth] = init(obj.TruthHistory,truths);
            obj.Truths = reshape(truths(sortIdx),[],1);
            obj.TruthHistory.Current.DetectableStatus(1:end) = getDetectableStatus(obj, obj.Truths);
        end
        
        function analyzeDeletedTracks(obj, iDeletedTrack)
            % get TrackIDs that were previously established            
            deletions = obj.TrackHistory.Deleted;
            established = ~deletions.FalseStatus(iDeletedTrack);
            trackID = deletions.TrackID(iDeletedTrack(established));

            % deassign them
            deassignTrack(obj, trackID);
            
            % remove assignment
            deletions.TruthID(1:end) = obj.InvalidTruthID;
        end
        
        function deassignTrack(obj, deassignedTrackID)
            % break all current truths that match the deassigned TrackID
            truths = obj.TruthHistory.Current;
            [exists, iTruthBreak] = ismember(deassignedTrackID, truths.TrackID);
            iTruthBreak = iTruthBreak(exists);
            truths.BreakStatus(iTruthBreak) = true;
            truths.BreakCount(iTruthBreak) = truths.BreakCount(iTruthBreak) + 1;
            truths.TrackID(iTruthBreak) = obj.InvalidTrackID;

            % clear redundancy status of earliest reported tracks that
            % match these truthIDs and clear break status of the
            % corresponding truths
            tracks = obj.TrackHistory.Current;
            iRedundantTracks = find(tracks.RedundancyStatus);
            [redundant, iFirstRedundant] = ismember(truths.TruthID(iTruthBreak), tracks.TruthID(iRedundantTracks));
            tracks.RedundancyStatus(iRedundantTracks(iFirstRedundant(redundant))) = false;
            truths.TrackID(iTruthBreak(redundant)) = tracks.TrackID(iRedundantTracks(iFirstRedundant(redundant)));
            truths.BreakStatus(iTruthBreak(redundant)) = false;
        end
        
        function analyzeExistingTracks(obj, iExistingTrack, iExistingTruth, iDeletedTruth)
            currentTracks = obj.TrackHistory.Current;
            
            % find false tracks
            falseTrack = currentTracks.FalseStatus(iExistingTrack);
            
            % check the remaining tracks first
            processDeletedAssignments(obj, iExistingTrack(~falseTrack), iDeletedTruth);
            processCurrentAssignments(obj, iExistingTrack(~falseTrack), iExistingTruth);
            
            % then try assigning false tracks without for track breaks
            attemptAssignment(obj, iExistingTrack(falseTrack), false);
        end
        
        function processDeletedAssignments(obj, iAssignedTrack, iDeletedTruth)
            currentTracks = obj.TrackHistory.Current;
            deletedTruths = obj.TruthHistory.Deleted;
            
            % if the assigned truth was deleted inside the coverage area
            % mark this track deleted
            [hasDeletedTruth, iTruth] = ismember(currentTracks.TruthID(iAssignedTrack), ...
                                                 deletedTruths.TruthID(iDeletedTruth));
            currentTracks.DeletionStatus(iAssignedTrack(hasDeletedTruth)) = ...
               deletedTruths.DetectableStatus(iDeletedTruth(iTruth(hasDeletedTruth)));
           
            % try to reassign without checking for track breaks
            attemptAssignment(obj, iAssignedTrack(hasDeletedTruth), false);
        end
        
        function processCurrentAssignments(obj, iAssignedTrack, iExistingTruth)
            currentTracks = obj.TrackHistory.Current;
            currentTruths = obj.TruthHistory.Current;
            
            % from all tracks that have an assignment to an existing truth
            [hasCurrentTruth, iTruth] = ismember(currentTracks.TruthID(iAssignedTrack), ...
                                                 currentTruths.TruthID(iExistingTruth));
                                             
            % find tracks that are diverging
            [iDiverging,iNonDiverging] = findDivergingTracks(obj, ...
                iAssignedTrack(hasCurrentTruth), ...
                iExistingTruth(iTruth(hasCurrentTruth)));
                                              
            % mark divergences
            iNewlyDiverging = iDiverging(~currentTracks.DivergenceStatus(iDiverging));
            currentTracks.DivergenceCount(iNewlyDiverging) = currentTracks.DivergenceCount(iNewlyDiverging) + 1;
            currentTracks.DivergenceStatus(iNewlyDiverging) = true;
            currentTracks.DivergenceStatus(iNonDiverging) = false;
            
            % try to reassign, but check for track breaks
            attemptAssignment(obj, iDiverging, true);
        end
        
        function [iDiverging,iNonDiverging] = findDivergingTracks(obj, iTrack, iTruth)
            if isempty(iTrack)
                iDiverging = zeros(0,1);
                iNonDiverging = zeros(0,1);
            else
                dist = getDivergenceDistance(obj, obj.Tracks(iTrack), obj.Truths(iTruth));
                isDiverging = dist > obj.DivergenceThreshold;
                iDiverging = iTrack(isDiverging);
                iNonDiverging = iTrack(~isDiverging);
            end
        end
        
        function dist = getDivergenceDistance(obj, tracks, truths)
            dist = zeros(numel(tracks),1);
            for i=1:numel(tracks)
               dist(i) = obj.DivergenceDistanceFcn(tracks(i), truths(i));
            end
        end       
        
        function analyzeNewTracks(obj, iNewTracks)
            % try to assign without checking for track breaks
            attemptAssignment(obj, iNewTracks, false);
        end

        function attemptAssignment(obj, iNewTracks, checkForBreak)
            if ~isempty(iNewTracks) && ~isempty(obj.Truths)
                dist = getAssignmentDistance(obj, obj.Tracks(iNewTracks), obj.Truths);
                [bdist, iBestTruth] = min(dist,[],2);
                iAssoc = find(bdist<obj.AssignmentThreshold);
                assignTracks(obj, iNewTracks(iAssoc), iBestTruth(iAssoc),checkForBreak);
            end
        end
        
        function dist = getAssignmentDistance(obj, tracks, truths)
            dist = zeros(numel(tracks),numel(truths));
            for i=1:numel(tracks)
                for j=1:numel(truths)
                    d = obj.AssignmentDistanceFcn(tracks(i), truths(j));
                    dist(i,j) = d;
                end
            end
        end
        
        function assignTracks(obj, iTracks, iTruths, checkForBreak)
            tracks = obj.TrackHistory.Current;
            truths = obj.TruthHistory.Current;

            assert(numel(iTracks)==numel(iTruths))
            
            % increment swap when track has a new truth            
            iTrackSwaps = iTracks(~tracks.FalseStatus(iTracks) & ...
                                   tracks.TruthID(iTracks) ~= truths.TruthID(iTruths)); 
                               
            tracks.SwapCount(iTrackSwaps) = tracks.SwapCount(iTrackSwaps) + 1;

            if checkForBreak
                % reassignment of a diverging track back to its pre-existing
                % track should not count as a track break
                iCandidates = iTracks(truths.TrackID(iTruths) ~= tracks.TrackID(iTracks));
                deassignTrack(obj, tracks.TrackID(iCandidates));
            end
            
            % find newly established truths and their corresponding tracks
            iTruthUnestablished = iTruths(~truths.EstablishmentStatus(iTruths));
            iTrackUnestablished = iTracks(~truths.EstablishmentStatus(iTruths));

            % increment establishment count
            truths.EstablishmentCount(iTruthUnestablished) = truths.EstablishmentCount(iTruthUnestablished) + 1;
            
            % mark the track of this association and clear its redundancy
            truths.TrackID(iTruthUnestablished) = tracks.TrackID(iTrackUnestablished);
            tracks.RedundancyStatus(iTrackUnestablished) = false;

            % find newly healed truths and their corresponding tracks
            iTruthHealed = iTruths(truths.BreakStatus(iTruths));
            iTrackHealed = iTracks(truths.BreakStatus(iTruths));
            
            % mark the track of this association and clear its redundancy
            truths.TrackID(iTruthHealed) = tracks.TrackID(iTrackHealed);
            tracks.RedundancyStatus(iTrackHealed) = false;
            
            % Mark status for each assigned track
            tracks.DivergenceStatus(iTracks) = false;
            tracks.DeletionStatus(iTracks) = false;
            tracks.FalseStatus(iTracks) = false;
            tracks.TruthID(iTracks) = truths.TruthID(iTruths);

            % establish the truth
            truths.EstablishmentStatus(iTruths) = true;
            truths.BreakStatus(iTruths) = false;
            
            % if not the trackID of the corresponding truth
            iTrackRedundant = iTracks(truths.TrackID(iTruths) ~= tracks.TrackID(iTracks));
            tracks.RedundancyStatus(iTrackRedundant) = true;
            tracks.RedundancyCount(iTrackRedundant) = tracks.RedundancyCount(iTrackRedundant) + 1;
        end
        
        function detStatus = getDetectableStatus(obj, truths)
            detStatus = obj.IsInsideCoverageAreaFcn(truths);
            if numel(detStatus)~=numel(truths) || ~islogical(detStatus)
                error(message('fusion:trackAssignmentMetrics:InvalidCoverageAreaFcn',char(obj.IsInsideCoverageAreaFcn(truths))));
            end
        end
    end    
    
    methods (Static, Access = {?matlab.unittest.TestCase})
        function perr = abserrposca(track,truth)
            perr = abserrposca(track,truth);
        end
        function perr = abserrposct(track,truth)
            perr = abserrposct(track,truth);
        end
        function perr = abserrposcv(track,truth)
            perr = abserrposcv(track,truth);
        end
        function verr = abserrvelca(track,truth)
            verr = abserrvelca(track,truth);
        end
        function verr = abserrvelct(track,truth)
            verr = abserrvelct(track,truth);
        end
        function verr = abserrvelcv(track,truth)
            verr = abserrvelcv(track,truth);
        end
         function perr = neesposca(track,truth)
            perr = neesposca(track,truth);
        end
        function perr = neesposct(track,truth)
            perr = neesposct(track,truth);
        end
        function perr = neesposcv(track,truth)
            perr = neesposcv(track,truth);
        end
        function verr = neesvelca(track,truth)
            verr = neesvelca(track,truth);
        end
        function verr = neesvelct(track,truth)
            verr = neesvelct(track,truth);
        end
        function verr = neesvelcv(track,truth)
            verr = neesvelcv(track,truth);
        end
    end
    
end
