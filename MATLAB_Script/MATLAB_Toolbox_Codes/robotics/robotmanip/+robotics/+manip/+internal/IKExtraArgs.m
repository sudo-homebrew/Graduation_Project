classdef IKExtraArgs < handle
    %This class is for internal use only. It may be removed in the future.
    
    %IKEXTRAARGS Extra arguments that inverseKinematics have to pass to the
    %   NLP solvers.
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties
        Robot
        WeightMatrix
        Limits
        BodyName
        Tform
        ErrTemp
        CostTemp
        GradTemp
    end


end

