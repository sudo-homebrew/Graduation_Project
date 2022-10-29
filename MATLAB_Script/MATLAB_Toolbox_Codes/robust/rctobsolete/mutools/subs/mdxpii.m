% function out = mdxpii(in,depth,row,col)

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.

function out = mdxpii(in,depth,row,col)
    out = xpii(xpii(xpii(in,depth),row),col);