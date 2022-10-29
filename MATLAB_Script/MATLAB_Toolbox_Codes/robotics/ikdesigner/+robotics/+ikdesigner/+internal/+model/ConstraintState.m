classdef ConstraintState < uint8
    %This class is for internal use only and may be removed in a future release

    %ConstraintState Enumeration class for the IK solver constraint state

    %   Copyright 2021-2022 The MathWorks, Inc.

    enumeration
        %Disabled The constraint is disabled, so it will not be included in the solver
        %   When the constraint is disabled, it cannot pass/fail because it
        %   is not part of the solver computation.
        Disabled (0)

        %Pass The constraint is enabled and satisfied
        %   When the constraint is enabled and the solver reaches a pose
        %   that satisfies the constraint, the state is passing.
        Pass    (1)

        %Fail The constraint is enabled but not satisfied
        %   When the constraint is enabled and the solver reaches a pose
        %   that does not satisfy the constraint, the state is failing.
        Fail    (2)

        %Unset The constraint is enabled but the satisfaction is not known
        %   When the constraint is enabled but no solver data is given
        %   (i.e. the solver has not yet computed a solution that includes
        %   this constraint), then its state is unset.
        Unset   (3)
    end

    methods (Static)
        function isEnabled = checkIfEnabled(state)
            %checkIfEnabled Verify whether a constraint state is enabled or disabled
            %   There is one disabled state and three enabled states. This
            %   check returns isEnabled if the state is not Disabled.

            isEnabled = ~(state == robotics.ikdesigner.internal.model.ConstraintState.Disabled);
        end
    end
end

