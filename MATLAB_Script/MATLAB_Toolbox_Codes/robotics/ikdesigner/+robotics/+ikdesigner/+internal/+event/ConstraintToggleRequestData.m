classdef (ConstructOnLoad) ConstraintToggleRequestData < event.EventData
%This class is for internal use only and may be removed in a future release.

%ConstraintToggleRequestData Event data for enabling and disabling constraints

%   Copyright 2021 The MathWorks, Inc.

    properties
        %ConstraintKeysToEnable Keys of the constraints (in the constraints map) to be enabled
        %   When enabled, constraints are active during the solver
        %   calculation.
        ConstraintKeysToEnable

        %ConstraintKeysToDisable Keys of the constraints (in the constraints map) to be disabled
        %   When disabled, constraints are not included during the solver
        %   calculation.
        ConstraintKeysToDisable
    end

    methods
        function data = ConstraintToggleRequestData(keysToEnable, keysToDisable)

            data.ConstraintKeysToEnable = keysToEnable;
            data.ConstraintKeysToDisable = keysToDisable;
        end
    end
end