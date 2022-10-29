classdef ConfigMapEntry
%This class is for internal use only and may be removed in a future release.

%ConfigMapEntry Class that defines structure of the values in the configurations model map

%   Copyright 2021 The MathWorks, Inc.    
    properties
        %Configuration
        Configuration

        %Name Label associated with the configuration
        Name = string.empty;

        %IKState Structure that specifies the ik state and solution info
        IKState = []

        %CollisionState Enumeration indicating the collision state (evaluated/pass/fail)
        CollisionState = robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated;

        %CollisionRawData Logical matrix that indicates whether any two bodies are colliding
        CollisionRawData = [];
    end
    
    methods
        function obj = ConfigMapEntry(config, name)
            %ConfigMapEntry Constructor

            obj.Configuration = config;
            obj.Name = name;
            obj.IKState = robotics.ikdesigner.internal.model.IKState([], string.empty);
        end
    end
end

