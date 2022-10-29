function validateValidatorVehicleCostmap(validator, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateValidatorVehicleCostmap Validate a validatorVehicleCostmap object

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    validateattributes(validator, {'validatorVehicleCostmap'}, {"nonempty", "scalar"}, ...
                       fcnName, varName); %#ok<CLARRSTR>

end
