% Copyright 2003-2004 The MathWorks, Inc.

function [idx1,idx2] = smidx(nr,r,c,dim)
%   gets MATLAB indices of a symmetric matrix, of dimension DIM,
%   starting at row R, column C, of a matrix with NR rows.
%   Order of indices returned in IDX scheme is goofy, but
%   consistent with LMILAB, as shown below (4x4) example:
%
%   1  2  4  7
%   2  3  5  8
%   4  5  6  9
%   7  8  9  10
%
%   Hence, using MATLAB indexing, we get
%   >> [idx1`,idx2] = smidx(4,1,1,4)
%       idx1 = [1;5;6;9;10;11;13;14;15;16]
%       idx2 = [1;2;6;3;7;11;4;8;12;16]

    len = dim*(dim+1)/2;
    idx1 = zeros(len,1);
    idx2 = zeros(len,1);
    loc = 1;
    for i=1:dim
        col = c + i - 1;
        rows = r:r+i-1;
        idx1(loc:loc+i-1,1) = rows' + nr*(col-1);
%       Acceptable code
        row = r + i - 1;
        cols = c:c+i-1;
        idx2(loc:loc+i-1,1) = row + nr*(cols'-1);
        % UPDATE LOC 
        loc = loc + length(rows);
    end

