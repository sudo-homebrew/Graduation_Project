function validateValidatorOccupancyMap(validator, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateValidatorOccupancyMap Validate a validatorOccupancyMap object

%   Copyright 2019 The MathWorks, Inc.

%#codegen
    validateattributes(validator, {'validatorOccupancyMap'}, {"nonempty", "scalar"}, ...
                       fcnName, varName); %#ok<CLARRSTR>

end
