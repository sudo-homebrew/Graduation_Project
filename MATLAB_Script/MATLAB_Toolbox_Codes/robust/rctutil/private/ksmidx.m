% Copyright 2003-2004 The MathWorks, Inc.

function [idx1,idx2] = ksmidx(nr,r,c,dim,nk)
%   gets MATLAB indices of a kronekared-symmetric matrix, of dimension DIM,
%   kronekared by NK,
%   starting at row R, column C, of a matrix with NR rows.

    len = dim*(dim+1)/2;
    idx1 = zeros(len,1);
    idx2 = zeros(len,1);
    rcinc = nk*(0:dim-1);
    loc = 1;
    for i=1:dim
        rows = r + rcinc(1:i);
        col = c + rcinc(i);
        idx1(loc:loc+i-1,1) = rows' + nr*(col-1);
        row = r + rcinc(i);
        cols = c + rcinc(1:i);
        idx2(loc:loc+i-1,1) = row + nr*(cols'-1);
        % UPDATE LOC 
        loc = loc + length(rows);
    end

