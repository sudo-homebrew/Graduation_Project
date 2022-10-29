function z = zeromax(x,y)
%ZEROMAX Return the maximum value in the input and zero if empty
%
%   This function is for internal use only and may be removed in a future
%   release of MATLAB

%   Copyright 2018 The MathWorks, Inc.

    z = max(vertcat(x,y));
    if isempty(z)
        z = 0;
    end
end