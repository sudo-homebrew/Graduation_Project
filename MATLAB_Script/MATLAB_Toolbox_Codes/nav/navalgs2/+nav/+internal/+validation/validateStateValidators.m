function validateStateValidators(validator, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateStateValidators Validate a validatorOccupancyMap and
%   validatorVehicleCostmap objects. It is being used by pathmetrics.m

%   Copyright 2019 The MathWorks, Inc.

    validateattributes(validator, {'validatorOccupancyMap', 'validatorVehicleCostmap'}, {"nonempty", "scalar"}, ...
                       fcnName, varName); %#ok<CLARRSTR>

end