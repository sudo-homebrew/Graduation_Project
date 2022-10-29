classdef trackBranchHistory < matlab.System
% trackBranchHistory Track-oriented MHT branching and branch history
%
%   b = trackBranchHistory creates a track-oriented branch history
%   manager. The branch history manager is responsible for creating track
%   branches, based on the result of the assignment of detections to
%   branches. It maintains the branch history, including unique IDs for
%   each branch and track.
%   
%   b = trackBranchHistory('Name', value) creates a trackBranchHistory
%   object by specifying its properties as name-value pair arguments.
%   Unspecified properties have default values. See the list of properties
%   below.
%
%   Step method syntax:
%
%   history = step(b,assignments,unassignedTracks,unassignedDetections,originatingSensor)
%   returns the branch history, history, based on the results of the
%   assignment algorithm (assignments, unassignedTracks, and
%   unassignedDetections) and originatingSensor. 
%   The list of assignments is a P-by-2 uint32 matrix, where P is the
%   number of assignments. The first column lists branch indices and the
%   second column lists detection indices.
%   The list of unassignedTracks is a Q-by-1 uint32 vector, where Q is the
%   number of unassigned track branches.
%   The list of unassignedDetections is an R-by-1 uint32 vector, where R is
%   the number of unassigned detections.
%   The vector originatingSensor provides the SensorIndex of each
%   detection. The i-th element of this vector corresponds to the sensor
%   index of detection i.
%
%   The output, history, is a matrix with 3+D*S columns, where D is the
%   number of scans (history depth) kept and S is the maximum number of
%   sensors. Each row of the history matrix represents a unique track
%   branch.
%   The first 3 columns are:
%   TrackID  - the track ID associated with this branch. All track branches
%              that are assumed to originate from the same target have the
%              same Track ID. A new track ID is used for every branch that
%              originates from an unassigned detection.
%   ParentID - the ID of the parent branch, from which this new branch is
%              created. Branches that were created from the same parent
%              have the same ParentID.
%   BranchID - the current ID of the track branch. This value is unique for
%              each track branch and every branch created from an
%              unassigned detection or an assignment gets a new branch ID.
%   The following D*S columns are organized in the following way:
%   |       Scan N      |      Scan N-1     |...|     Scan N-D      |
%   |Sen-1 Sen-2...Sen-S|Sen-1 Sen-2...Sen-S|...|Sen-1 Sen-2...Sen-S|
%
%   trackBranchHistory properties:
%     MaxNumSensors       - Define the maximum number of sensors
%     MaxNumHistoryScans  - Define the maximum number of scans kept in the 
%                           track history
%     MaxNumTracks        - Define the maximum number of tracks
%     MaxNumTrackBranches - Define the maximum number of branches
%                           (hypotheses) per track
%
%   trackBranchHistory methods:
%     step       - Creates, updates, and deletes the tracks
%     getHistory - Returns the current history as table
%     release    - Allows property value and input characteristics changes
%     clone      - Creates a copy of the trackBranchHistory
%     isLocked   - Locked status (logical)
%     reset      - Resets states of the trackBranchHistory
%
%   EXAMPLE: Branch tracks based on assignment results
%
%   % Create a trackBranchHistory that handles four sensors and two scans
%   b = trackBranchHistory('MaxNumSensors',4,'MaxNumHistoryScans',2)
%   
%   % In the first update there should be no prior branches, there are only
%   % unassigned detections
%   emptyAssignment = zeros(0,2,'uint32');
%   emptyUnassignment = zeros(0,1,'uint32');
%   originatingSensor = [1 1 2];
%   history = b(emptyAssignment,emptyUnassignment,uint32([1;2;3]),...
%       originatingSensor);
%
%   % For an easy representation of the history, use getHistory
%   getHistory(b)
%
%   % Update with multiple assignments and unassignments. In a
%   % track-oriented MHT, the same branch may be assigned to multiple
%   % detections or unassigned, and the same detection may be assigned to
%   % multiple branches or unassigned. The trackBranchHistory creates a
%   % branch history corresponding to each assignment and unassignment.
%   assigned = uint32([1 1; 1 2; 2 1; 2 2]);
%   unasTrs = uint32([1;3]);
%   unasDets = uint32([1;2;3]);
%   history = b(assigned,unasTrs,unasDets,originatingSensor);
%   getHistory(b)
%
%   See also: trackerTOMHT, assignTOMHT, clusterTrackBranches

%   References:
%   [1] J.R. Werthmann, "A Step-by-Step Description of a Computationally
%       Efficient Version of Multiple Hypothesis Tracking", SPIE Vol. 1698
%       Signal and Processing of Small Targets, pp 288-300, 1992.

% Copyright 2018-2021 The MathWorks, Inc.

%#codegen

    % Public, tunable properties
    properties(Nontunable)
        %MaxNumSensors Maximum number of sensors
        %   Set the maximum number of sensors as a positive real integer.
        %
        %   Default: 20
        MaxNumSensors (1, 1) {mustBePositive, mustBeInteger} = 20
        
        %MaxNumHistoryScans  Maximum number of scans maintained in the branch history
        %   Set the maximum number of scans maintained in the branch
        %   history.Track history is typically between 2 and 6. Higher
        %   values will increase computational load.
        %   
        %   Default: 4
        MaxNumHistoryScans (1, 1) {mustBePositive, mustBeInteger} = 4
        
        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 200
        MaxNumTracks (1, 1) {mustBePositive, mustBeInteger} = 200
        
        %MaxNumTrackBranches - Maximum number of track branches per track
        %   Set the maximum number of track branches (hypotheses) allowed
        %   for each track. Higher values will increase computational load.
        MaxNumTrackBranches (1, 1) {mustBePositive, mustBeInteger} = 3
    end

    properties(Dependent, Access={?trackBranchHistory, ?trackerTOMHT, ?matlab.unittest.TestCase})
        % Keeps the track history. It is modified at each step
        TrackHistory
    end

    properties(Access = protected)
        pLastBranchID
        pLastTrackID
        pTrackHistory
        pUsedHistory
    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods
        function obj = trackBranchHistory(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
            obj.pTrackHistory = zeros(obj.MaxNumTracks*obj.MaxNumTrackBranches,...
                3+(obj.MaxNumSensors*obj.MaxNumHistoryScans),'uint32');
            obj.pUsedHistory = false(obj.MaxNumTracks*obj.MaxNumTrackBranches,1);
        end
        function set.TrackHistory(obj,value)
            numCols = 3+obj.MaxNumHistoryScans*obj.MaxNumSensors;
            validateattributes(value,{'numeric'},{'real','nonsparse',...
                'nonnegative','ncols', numCols},...
                mfilename,'TrackHistory')
            obj.pUsedHistory = false(obj.MaxNumTracks*obj.MaxNumTrackBranches,1);
            obj.pUsedHistory(1:size(value,1)) = true;
            obj.pTrackHistory(1:size(value,1),1:numCols) = cast(value,'uint32');
        end
        function value = get.TrackHistory(obj)
            if coder.internal.is_defined(obj.pTrackHistory)
                value = obj.pTrackHistory(obj.pUsedHistory,:);
            else
                value = zeros(0,3+obj.MaxNumHistoryScans*obj.MaxNumSensors,'uint32');
            end
        end
        function history = getHistory(obj, varargin)
            %getHistory Return the branch history managed by the object
            %   history = getHistory(obj) returns the branch history
            %   managed by the object tabulated as a table.
            %   history = getHistory(obj, FORMAT) allows you to specify the
            %   format of the output with the following values:
            %   'Table'  - returns the history in a table (default).
            %   'Matrix' - returns the history as a matrix, same as the
            %              output of the step method.
            
            narginchk(1,2)
            if numel(varargin) == 0
                format = 1; % Table
            else
                s = validatestring(varargin{1},{'Matrix','Table'},mfilename,'FORMAT');
                if strcmpi(s,'Matrix')
                    format = 2; % Matrix
                else
                    format = 1; % Table
                end
            end
                
            h = obj.TrackHistory;
            
            if format==1
                numNames = size(h,2);
                names = cell(1,numNames);
                names{1} = 'TrackID';
                names{2} = 'ParentID';
                names{3} = 'BranchID';
                for i = 1:obj.MaxNumHistoryScans
                    for j = 1:obj.MaxNumSensors
                        k = (i-1)*obj.MaxNumSensors + j;
                        names{k+3} = ['Scan',num2str(obj.MaxNumHistoryScans-i+1),'Sensor',num2str(j)];
                    end
                end
                history = array2table(h, 'VariableNames', names);
                for i = obj.MaxNumHistoryScans:-1:1
                    history = mergevars(history, 3+obj.MaxNumSensors*(i-1)+(1:obj.MaxNumSensors),...
                        'NewVariableName',['Scan',num2str(obj.MaxNumHistoryScans - i+1)],...
                        'MergeAsTable',true);
                    history.(['Scan',num2str(obj.MaxNumHistoryScans - i+1)]).Properties.VariableNames = ...
                        strrep(history.(['Scan',num2str(obj.MaxNumHistoryScans - i+1)]).Properties.VariableNames,...
                        ['Scan',num2str(obj.MaxNumHistoryScans - i+1)],'');
                end
            else
                history = h;
            end
        end
    end
    
    methods(Access = {?trackBranchHistory, ?trackerTOMHT})
        function [trID, brID] = getLastIDs(obj)
            trID = obj.pLastTrackID;
            brID = obj.pLastBranchID;
        end
        function setLastIDs(obj,trID,brID)
            obj.pLastTrackID = trID;
            obj.pLastBranchID = brID;
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.pTrackHistory = zeros(obj.MaxNumTracks*obj.MaxNumTrackBranches,...
                3+(obj.MaxNumSensors*obj.MaxNumHistoryScans),'uint32');
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.TrackHistory  = zeros(0,3+obj.MaxNumSensors*(obj.MaxNumHistoryScans),'uint32');
            obj.pLastBranchID = uint32(0);
            obj.pLastTrackID  = uint32(0);
            obj.pUsedHistory = false(obj.MaxNumTracks*obj.MaxNumTrackBranches,1);
        end

        function validateInputsImpl(~,assignments,unassignedTracks,unassignedDetections,originatingSensor)
            % Validate inputs to the step method at initialization
            validateattributes(assignments,{'numeric'},...
                {'real','nonsparse','positive','integer','ncols',2},...
                mfilename,'assignments')
            validateattributes(unassignedTracks,{'numeric'},...
                {'real','nonsparse','positive','integer','column'}, ...
                mfilename,'unassignedTracks')
            validateattributes(unassignedDetections,{'numeric'},...
                {'real','nonsparse','positive','integer','column'}, ...
                mfilename,'unassignedDetections')
            validateattributes(originatingSensor,{'numeric'},...
                {'real','nonsparse','positive','integer'},...
                mfilename,'originatingSensor');
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.pLastBranchID = s.pLastBranchID;
            obj.pLastTrackID  = s.pLastTrackID;
            obj.pTrackHistory = s.pTrackHistory;
            obj.pUsedHistory  = s.pUsedHistory;

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.pLastBranchID = obj.pLastBranchID;
            s.pLastTrackID  = obj.pLastTrackID;
            s.pTrackHistory = obj.pTrackHistory;
            s.pUsedHistory  = obj.pUsedHistory;
        end
        %------------------------------------------------------------------

        function updatedHistory = stepImpl(obj,assignments,...
                unassignedTracks,unassignedDetections,originatingSensor)
            % Update track hypotheses history using the information in
            % assignments, unassignedTracks and unassignedDetections. 
            % The following options are possible:
            %   1. Unassigned tracks: 
            %      - Shift the track history by one step.
            %   2. Unassigned detections:
            %      - Create a new track hypothesis for each detection
            %      - Combinations of assignments from different sensors?
            %   3. Assignments:
            %      - Shift the track history by one step.
            %      - Each assignment should create a new hypothesis.
            %      - Combinations of assignments from different sensors?
            
            % Extract the history of the current live tracks.
            history = obj.TrackHistory;
           
            % Case 1: Unassigned tracks
            unasTrackHistory = unassignedTracksHistory(obj, history, unassignedTracks);
            
            % Case 2: Unassigned detections
            unasDetsHistory = unassignedDetectionsHistory(obj, history, unassignedDetections,originatingSensor);
            
            % Case 3: Assigned tracks
            assignedHistory = assignedTracksHistory(obj, history, assignments, originatingSensor);
            updatedHistory = [unasTrackHistory;unasDetsHistory;assignedHistory];
            obj.TrackHistory = updatedHistory;
        end
        %------------------------------------------------------------------
        
        function updatedHistory = unassignedTracksHistory(obj, history, unassignedTracks)
            % For unassigned tracks, the TrackID, ParentID, and BranchID
            % remain the same. The only difference is that the scans
            % history is shifted by one scan from the j-th scan to the
            % j-1-th scan. ParentID comes from the unassignedTrack IDs.
            
            % Allocate memory
            rowIndices = findTracksInHistory(obj, history, unassignedTracks);
            
            numTracks = numel(rowIndices);
            numSensors = obj.MaxNumSensors;
            historySize = size(history,2);
            updatedHistory = zeros(numTracks,historySize,'uint32');
            
            % Copy the TrackID and BranchID
            updatedHistory(1:numTracks,[1,3]) = history(rowIndices,[1,3]);
            
            % Take the ParentID from the unassignedTracks
            updatedHistory(1:numTracks,2) = unassignedTracks(:);
            
            % Shift the next (D-1)*S columns to the right (past). Leave the
            % most recent scan as 'NaN' values.
            updatedHistory(1:numTracks,(4+numSensors):end) = history(rowIndices,4:end-numSensors);
        end
        %------------------------------------------------------------------
        
        function rowIndices = findTracksInHistory(~, history, unassignedTracks)
            n = numel(unassignedTracks);
            rowIndices = zeros(n,1,'uint32');
            trIDs = history(:,3);
            for i = 1:n
                rowInd = find(unassignedTracks(i)==trIDs,1,'first');
                coder.internal.errorIf(isempty(rowInd),'fusion:trackerTOMHT:TrackIDNotFound',unassignedTracks(i));
                rowIndices(i) = rowInd;
            end
        end
        %------------------------------------------------------------------
        
        function updatedHistory = unassignedDetectionsHistory(obj, history, unassignedDetections, originatingSensor)
            % Each unassigned detection is assumed to the root of a new
            % track. Thus, each one gets a unique track ID.
            % Since it has no parent, the parent index remains a 'NaN'
            numTracks = numel(unassignedDetections);
            historySize = size(history,2);
            updatedHistory = zeros(numTracks,historySize,'uint32');
            
            % Assign new track IDs
            updatedHistory(1:numTracks,1) = obj.pLastTrackID+uint32(1:numTracks);
            obj.pLastTrackID = obj.pLastTrackID+numTracks;
            
            % Assign new branch indices
            updatedHistory(1:numTracks,3) = obj.pLastBranchID+uint32(1:numTracks);
            obj.pLastBranchID = obj.pLastBranchID+numTracks;
            
            % Record sensor and detection ID
            for i = 1:numTracks
                updatedHistory(i,3+originatingSensor(unassignedDetections(i))) = unassignedDetections(i);
            end
        end
        %------------------------------------------------------------------
        
        function updatedHistory = assignedTracksHistory(obj, history, assignments, originatingSensor)
            % Each combination of (track,detection) assignment creates a
            % new branch with the following relationship:
            %   TrackID  - Same as the assigned track's TrackID.
            %   ParentID - Same as the assigned track's BranchID.
            %   BranchID - New and unique for each branch.
            % The scan history is shifted one scan to the right (past). The
            % most recent scan saves the information from the assignment.
            % We allow multiple assignments if detections have different
            % originatingSensor indices.
            assignedTracks = unique(assignments(:,1));
            numTracks = size(assignedTracks,1);
            numSensors = obj.MaxNumSensors;
            historyInCell = cell(numTracks,1);
            
            for i = 1:numTracks
                % Find all the assignments for that track
                trackIFlags = (assignments(:,1)==assignedTracks(i));
                trackDets = assignments(trackIFlags,2);
                if ~any(trackDets)
                    historyInCell{i} = zeros(0,size(history,2),'uint32');
                    continue
                end
                
                % Find originating sensor for these dets
                origSens = originatingSensor(trackDets);
                uniqueOrigSens = unique(origSens);
                
                % Find all assignments combinations for the track
                assCombs = localAssignmentCombinations(trackDets, origSens);
                numCombs = size(assCombs,1);
                
                % Find the history row corresponding to this track ID
                trRow = find(assignedTracks(i) == history(:,3),1,'first');
                coder.internal.assert(isscalar(trRow),'fusion:trackerTOMHT:TrackIDNotFound',assignedTracks(i));
                
                % Allocate memory for all branches originating from this track
                historyInCell{i} = zeros(numCombs,size(history,2),'uint32');

                % Shift history one scan to the right (past)
                historyInCell{i}(1:numCombs,(4+numSensors):end) = repmat(history(trRow,4:end-numSensors),[numCombs,1]);
                
                % Copy TrackID to the branches originating from this branch
                % Copy ParentID to the branches originating from this branch
                historyInCell{i}(1:numCombs,1:2) = repmat(history(trRow,1:2:3),[numCombs,1]);
                
                % Add unique BranchID to each branch
                historyInCell{i}(1:numCombs,3) = (obj.pLastBranchID+1:obj.pLastBranchID+numCombs)';
                obj.pLastBranchID = obj.pLastBranchID + numCombs;
                
                % Add history from current scan
                historyInCell{i}(1:numCombs,3+uniqueOrigSens) = assCombs;
            end
            if coder.target('MATLAB')
                updatedHistory = cell2mat(historyInCell); 
            else
                if isempty(historyInCell)
                    updatedHistory = zeros(0,size(history,2),'uint32');
                else
                    updatedHistory = cell2matCG(historyInCell);
                end
            end
        end
    end
    
    methods(Static, Hidden)    
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

function trackDetCombs = localAssignmentCombinations(detIDs,originatingSensor)
%localFindAssignmentCombinations - assignment combinations in the track gate
% trackDetCombs = findAssignmentCombinations(detIDs,originatingSensor)
% calculates all the combinations of assignments of detections from
% different sensors that fall in the gate of a track. detIDs is an
% N-element array of detection IDs, and originatingSensor is an N-by-1
% array of the indices of the sensors that reported these detections.
%
% Example:
% --------
% detIDs = uint32([1;2;3;4;5]);
% originatingSensor = [1;1;2;2;3];
% outAssComb = findAssignmentCombinations(detIDs,originatingSensor)


uniqueOrigSensors = unique(originatingSensor);
numUniqueOrigSensors = numel(uniqueOrigSensors);
numDetsPerSensor = zeros(numUniqueOrigSensors,1);
c = cell(numUniqueOrigSensors,1);
for j = 1:numUniqueOrigSensors
    numDetsPerSensor(j) = sum(uniqueOrigSensors(j) == originatingSensor);
    c{j} = [NaN(1);double(detIDs((uniqueOrigSensors(j) == originatingSensor)))];
end
assComb = c{1};
coder.varsize('assComb',[Inf,Inf],[1 1]);
for j = 2:numUniqueOrigSensors
    numcjel = numel(c{j});
    lenAssComb = size(assComb,1);
    cjexpand = zeros(numcjel*lenAssComb,1);
    for k = 1:numcjel
        cjexpand((k-1)*lenAssComb+1:k*lenAssComb) = c{j}(k);
    end
    assComb = [repmat(assComb, [numcjel,1]),cjexpand];
end

% Output: the first row is all NaNs (trivial assignment)
trackDetCombs = assComb(2:end,:);
end

function updatedHistory = cell2matCG(historyInCell)
% Codegen redirect for cell2mat
totalNumRows = 0;
numCells = numel(historyInCell);
for i=1:numCells
    totalNumRows = totalNumRows + size(historyInCell{i},1);
end
numCols = size(historyInCell{1},2);
updatedHistory = zeros(totalNumRows,numCols,'uint32');

currentRow = 0;
for i=1:numCells
    numCellRows = size(historyInCell{i},1);
    updatedHistory(currentRow+1:currentRow+numCellRows,:) = historyInCell{i};
    currentRow = currentRow+numCellRows;
end
end
