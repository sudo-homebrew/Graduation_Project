function [endPts, middlePts] = raycastCells(p1, p2, rows, cols, resolution, gridLocation, rangeIsMax)
%This function is for internal use only. It may be removed in the future.

%RAYCASTCELLS Get cells along a line segment
%
%   [ENDPTS, MIDDLEPTS] = RAYCASTCELLS(P1, P2,  ROWS, COLS, RES, LOC) returns
%   cells along line segment(s) P1=[X1,Y1] to P2(i)=[X2,Y2] of the laser range beam.
%   The ENDPTS represent indices of cells touching all P2 and the MIDDLEPTS
%   represent all points between P1 and the set of P2, excluding ENDPTS.
%
%   [ENDPTS, MIDDLEPTS] = RAYCASTCELLS(P1, P2,  ROWS, COLS, RES, LOC, RANGEISMAX) 
%   returns cells along line segment(s) P1=[X1,Y1] to P2(i)=[X2,Y2] of the 
%   laser range beam. RANGEISMAX is an N-element boolean vector. When RANGEISMAX(i)
%   is true, point P2 is considered to be a max range reading, so points
%   touching P2(i) are appended to MIDDLEPTS rather than ENDPTS.

%   Endpoints are in world coordinate system and
%   can be floating point values. ROWS and COLS are map size in terms of
%   rows and columns, RES is the resolution of the grid cells in cells per
%   meter, LOC is the location of the lower left corner of the grid in
%   the world frame, and RANGEISMAX is an N-element boolean vector.
%   Input P1 (1x2) and P2 (Nx2) are vectors representing points on the grid.
%   X is Column index and Y is row index. 
%   When multiple end-points are provided, results for each ray are vertically
%   combined to form the outputs endPts, middlePts.
%   This algorithm is known as Digital Differential Analyzer (or DDA).

%   Copyright 2014-2019 The MathWorks, Inc.
%
%   Reference:
%   [1] "ARTS: Accelerated Ray-Tracing System,", Fujimoto, A.; Tanaka, T.;
%       Iwata, K., Computer Graphics and Applications, IEEE , vol.6, no.4,
%       pp.16,26, April 1986

%#codegen

% Do nothing if there are no endpoints provided
if isempty(p2)
    endPts = [];
    middlePts = [];
    return;
end

if nargin < 7
    rangeIsMax = false(size(p2,1),1);
end

% For simulation use the mex-file
if coder.target('MATLAB')
    % Run mex
    [endPts, middlePts] = ...
        fusion.internal.mex.raycastCells(p1, p2, rows, cols, resolution, gridLocation, rangeIsMax);
else
    % Run MATLAB-code
    [~, ~, endPts, middlePts] = ...
        fusion.internal.raycastCellsInternalImpl(p1, p2, rows, cols, resolution, gridLocation, rangeIsMax);
end
