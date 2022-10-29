classdef Data
    %DATA Resource data members
    %   List of static constants
    
    %   Copyright 2021-2022 The MathWorks, Inc.
    
    % Icon constants
    properties (Constant)
        %ICONSDIRECTORY Directory where standard toolstrip icons are stored
        ICONSDIRECTORY = fullfile(matlabroot,'toolbox','shared','controllib','general','resources','toolstrip_icons');

        %CUSTOMICONSDIRECTORY Directory for app-specific icons
        CUSTOMICONSDIRECTORY = fullfile(matlabroot,'toolbox','robotics','ikdesigner', 'icons');
    end

    % Solver constants
    properties (Constant)

        %DEFAULTSOLVERPARAMS
        DEFAULTSOLVERPARAMS = struct(...
            "MaxIterations", 50, ...
            "MaxTime", 5, ...
            "EnforceJointLimits", true, ...
            "AllowRandomRestart", true)

        %DEFAULTSOLVERALGORITHM
        DEFAULTSOLVERALGORITHM = "BFGSGradientProjection"

        %CONSTRAINTPASSTOLERANCE
        CONSTRAINTPASSTOLERANCE = eps(1)^(1/4)

        %MARKERPOSETARGETKEY Reserved key in the constraints map for storing the marker pose constraint
        %   The marker pose constraint is stored in the constraints map,
        %   but it used a reserved key that cannot overlap with a UUID
        %   value. Since this constraint cannot be removed, the value is
        %   persistent. This allows the key to be easily accessed from a
        %   constraints map in the model or views.
        MARKERPOSETARGETKEY = "markerposeconstraintkey"
    end

    % View constants
    properties (Constant)
        EULERCONVENTIONORDER = 'XYZ'
    end
end

