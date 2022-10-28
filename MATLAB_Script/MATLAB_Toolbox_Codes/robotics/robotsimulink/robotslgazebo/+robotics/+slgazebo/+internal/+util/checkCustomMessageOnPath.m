function ret = checkCustomMessageOnPath(fileName)
%This function is for internal use only. It may be removed in the future.

% checkCustomMessageOnPath checks input file exist or not for custom message
% support.

%   Copyright 2020 The MathWorks, Inc.

    if(exist(fileName,'file'))
        ret = true;
    else
        ret = false;
    end

end
