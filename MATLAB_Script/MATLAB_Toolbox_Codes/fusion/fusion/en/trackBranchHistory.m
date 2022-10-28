classdef trackBranchHistory< matlab.System
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
%     <a href="matlab:help matlab.System/reset   ">reset</a>      - Resets states of the trackBranchHistory
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

 
% Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=trackBranchHistory
            % Support name-value pair arguments when constructing object
        end

        function out=assignedTracksHistory(~) %#ok<STOUT>
            % Each combination of (track,detection) assignment creates a
            % new branch with the following relationship:
            %   TrackID  - Same as the assigned track's TrackID.
            %   ParentID - Same as the assigned track's BranchID.
            %   BranchID - New and unique for each branch.
            % The scan history is shifted one scan to the right (past). The
            % most recent scan saves the information from the assignment.
            % We allow multiple assignments if detections have different
            % originatingSensor indices.
        end

        function out=findTracksInHistory(~) %#ok<STOUT>
        end

        function out=getHistory(~) %#ok<STOUT>
            %getHistory Return the branch history managed by the object
            %   history = getHistory(obj) returns the branch history
            %   managed by the object tabulated as a table.
            %   history = getHistory(obj, FORMAT) allows you to specify the
            %   format of the output with the following values:
            %   'Table'  - returns the history in a table (default).
            %   'Matrix' - returns the history as a matrix, same as the
            %              output of the step method.
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Initialize / reset discrete-state properties
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Perform one-time calculations, such as computing constants
        end

        function out=stepImpl(~) %#ok<STOUT>
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
        end

        function out=unassignedDetectionsHistory(~) %#ok<STOUT>
            % Each unassigned detection is assumed to the root of a new
            % track. Thus, each one gets a unique track ID.
            % Since it has no parent, the parent index remains a 'NaN'
        end

        function out=unassignedTracksHistory(~) %#ok<STOUT>
            % For unassigned tracks, the TrackID, ParentID, and BranchID
            % remain the same. The only difference is that the scans
            % history is shifted by one scan from the j-th scan to the
            % j-1-th scan. ParentID comes from the unassignedTrack IDs.
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
            % Validate inputs to the step method at initialization
        end

    end
    properties
        %MaxNumHistoryScans  Maximum number of scans maintained in the branch history
        %   Set the maximum number of scans maintained in the branch
        %   history.Track history is typically between 2 and 6. Higher
        %   values will increase computational load.
        %   
        %   Default: 4
        MaxNumHistoryScans;

        %MaxNumSensors Maximum number of sensors
        %   Set the maximum number of sensors as a positive real integer.
        %
        %   Default: 20
        MaxNumSensors;

        %MaxNumTrackBranches - Maximum number of track branches per track
        %   Set the maximum number of track branches (hypotheses) allowed
        %   for each track. Higher values will increase computational load.
        MaxNumTrackBranches;

        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 200
        MaxNumTracks;

        pLastBranchID;

        pLastTrackID;

        pTrackHistory;

        pUsedHistory;

    end
end
