classdef IKState
%This class is for internal use only and may be removed in a future release

%IKState Value class that defines the structure of stored IK state values

%   Copyright 2021 The MathWorks, Inc.
    
    properties
        %Info Structure containing the info output of the solver
        %   When the solver is executed, it returns a solution and a set of
        %   solution details, stored as a struct. This property contains
        %   these details.
        Info

        %ConstraintKeys String array of keys for the constraints included in a given solver solution
        %   When the solver is executed, the info output includes a struct
        %   array of constraint violations that follows the order of
        %   constraints input into the solver. This property contains the
        %   array of keys corresponding to those constraints, in that
        %   order.
        ConstraintKeys
    end
    
    methods
        function obj = IKState(info,constraintKeys)
            %IKState Constructor

            obj.Info = info;
            obj.ConstraintKeys = constraintKeys;
        end
    end
end

