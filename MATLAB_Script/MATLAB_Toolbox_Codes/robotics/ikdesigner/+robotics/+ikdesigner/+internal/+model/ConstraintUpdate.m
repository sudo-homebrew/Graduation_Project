classdef ConstraintUpdate < uint8
%This class is for internal use only and may be removed in a future release

%ConstraintUpdate Enumeration class for the IK solver constraint update action
%   The constraint update is used to describe how an event is changing a
%   constraint.

%   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        %MapValuesChange The values in the constraints map are being changed
        %   This can occur when a constraint is modified, a new constraint
        %   is added, or an existing constraint is removed from the
        %   constraints map.
        MapValuesChange  (0)

        %StateChange The constraint state is changing
        %   This occurs when the constraint properties are consistent, but
        %   its state (as indicated in the ConstraintState enum) changes.
        StateChange      (1)

        %StateReset The constraint state is reset to default
        %   This means that the constraint state is reset, i.e. to unset.
        StateReset       (2)
    end
end

