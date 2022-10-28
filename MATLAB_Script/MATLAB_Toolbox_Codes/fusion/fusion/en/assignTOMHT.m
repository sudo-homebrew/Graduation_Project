%assignTOMHT Track-oriented multi-hypotheses tracking (TOMHT) assignment
%   [assignments, unassignedTracks, unassignedDetections] = assignTOMHT(costMatrix,costThreshold)
%   returns a list of assignments of detections to tracks, a list of
%   unassigned tracks, and a list of unassigned detections. The costMatrix
%   must be an M-by-N real matrix, where M is the number of tracks and N is
%   the number of detections.
%   costThreshold must be a 3-element finite, real, vector
%   [C1Gate,C2Gate,C3Gate], where C1Gate <= C2Gate <= C3Gate.
%   Let c = costMatrix(i,j). The following cases exist:
%
%   Case         | Track unassigned | Detection unassigned | Detection is assigned to track
%   -----------------------------------------------------------------------
%   c < C1       | No               | No                   | Yes
%   C1 <= c < C2 | Yes              | No                   | Yes
%   C2 <= c < C3 | Yes              | Yes                  | Yes
%   C3 <= c      | Yes              | Yes                  | No
%
%   The list of assignments is a P-by-2 uint32 matrix, where P is the
%   number of possible assignments. The first column lists the row indices
%   in the costMatrix input, corresponding to tracks, and the second column
%   lists the column indices, corresponding to detections.
%   The list of unassignedTracks is a Q-by-1 uint32 vector, where Q is the
%   number of unassigned tracks. The values correspond to the row indices.
%   The list of unassignedDetections is an R-by-1 uint32 vector, where R is
%   the number of unassigned detections. The values correspond to the
%   column indices.
%
%   Notes:
%   1. To allow each track to be unassigned, use C1Gate = 0.
%   2. To allow each detection to be unassigned, use C2Gate = 0.
%
%   % Example 1: Assignment with nonzero C1Gate and C2Gate
%   % ----------------------------------------------------
%   % Create a cost matrix that assigns:
%   %   Track 1 to detection 1 (in C1Gate) and detection 2 (in C2Gate).
%   %   Track 2 to detection 2 (in C2Gate) and detection 3 (in C3Gate).
%   %   Track 3 is unassigned.
%   %   Detection 4 is unassigned.
%   costMatrix = [4 9 200 Inf; 300 12 28 Inf; 32 100 210 1000]
%   costThresh = [5 10 30];
%
%   % Calculate the assignment
%   [assgns, unasTracks, unasDets] = assignTOMHT(costMatrix,costThresh)
%
%   % Tracks that are assigned detections within the C1Gate are not
%   % considered as unassigned. In this example, track 1.
%   % Detections that are assigned to tracks within the C2Gate are not
%   % considered as unassigned. In this example, detections 1 and 2.
%
%   % Example 2: Assignment with C1Gate=C2Gate=0
%   % ------------------------------------------
%   % Create a cost matrix that assigns:
%   %   Track 1 to detection 1 and detection 2.
%   %   Track 2 to detection 2 and detection 3.
%   % All the tracks are considered unassigned, because C1Gate is zero.
%   % All the detections are considered unassigned, because C2Gate is zero.
%   costMatrix = [4 9 200 Inf; 300 12 28 Inf; 32 100 210 1000]
%   costThresh = [0 0 30];
%
%   % Calculate the assignment
%   [assgns, unasTracks, unasDets] = assignTOMHT(costMatrix,costThresh)
%
%   See also: trackerTOMHT, trackBranchHistory

 
% Copyright 2018 The MathWorks, Inc.

