function prob = intLogoddsToProb(lookupTable, intSpace, logodds)
%This function is for internal use only. It may be removed in the future.

%intLogoddsToProb Convert int16 logodd map values to probability
%   This function is MEX'ed for MATLAB execution.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    offset = int32(intSpace(1))-1;
    prob = double(lookupTable(int32(logodds(:))-offset));
end