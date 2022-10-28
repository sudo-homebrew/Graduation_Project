classdef (ConstructOnLoad) CollisionStateData < event.EventData
    %This function is for internal use only and may be removed in a future release

    %CollisionStateData Event object for communicating collision state

    %   Copyright 2021 The MathWorks, Inc.

    properties
        %SceneObjectsMap Map of all the objects in the scene with structures as values
        SceneObjectsMap

        %Config Robot joint configuration
        Config

        %CollisionState Enumeration indicating tree collision state
        %   The overall collision state has three possible values:
        %   collision-free, in collision, or not evaluated.
        RBTCollisionState
    end

    methods
        function data = CollisionStateData(config, sceneObjectsMap, rbtCollisionState)

            arguments
                config              double
                sceneObjectsMap     containers.Map
                rbtCollisionState   robotics.ikdesigner.internal.model.RBTCollisionState
            end

            data.Config = config;
            data.SceneObjectsMap = sceneObjectsMap;
            data.RBTCollisionState = rbtCollisionState;
        end
    end
end