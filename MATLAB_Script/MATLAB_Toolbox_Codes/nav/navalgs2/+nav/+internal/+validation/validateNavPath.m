function validateNavPath(pathobj, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateNavPath Validate a navPath object

%   Copyright 2019 The MathWorks, Inc.

    validateattributes(pathobj, "navPath", {"nonempty", "scalar"}, ...
                       fcnName, varName); %#ok<CLARRSTR>

end
