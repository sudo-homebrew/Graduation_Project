% Copyright 2003-2004 The MathWorks, Inc.
    
function idx = kssmidx(nr,r,c,dim,nk)
%   gets MATLAB indices of a skew-symmetric matrix, of dimension DIM,
%   starting at row R, column C, of a matrix with NR rows.
%   Order of indices returned in IDX scheme is goofy, but
%   consistent with LMILAB, as shown below (4x4) example:
%
%   ? -1 -2 -4
%   1  ? -3 -5
%   2  3  ? -6
%   4  5  6  ?
%
%   Hence, using MATLAB indexing, we get
%   >> [idx] = ssmidx(4,1,1,4)
%       idx = [2;3;7;4;8;12]

    len = dim*(dim-1)/2;
    idx = zeros(len,1);
    loc = 1;
    rcinc = nk*(0:dim-1);
    for i=2:dim
        cols = c + rcinc(1:i-1);
        row = r + rcinc(i);
        idx(loc:loc+i-2,1) = row + nr*(cols'-1);
        loc = loc + length(cols);
    end   
