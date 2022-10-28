classdef CoreSceneState < robotics.ikdesigner.internal.model.Model
    %This class is for internal use only and may be removed in a future release

    %CoreSceneState Class containing the core state shared by models

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)

        %RigidBodyTree RigidBodyTree object
        RigidBodyTree
    end

    properties (SetAccess = ?robotics.ikdesigner.internal.model.SceneModel)
        
        %SceneObjectsMap A map containing all the objects in the scene
        %   The Scene objects map is a map of all possible objects in the
        %   scene. It contains the handle to the object, as well as any
        %   necessary descriptors, e.g. name for collision body. The map
        %   uses unique UUIDs as keys, so that any entry can be accessed by
        %   its key. In this way, any object in the scene -- rigid body or
        %   collision body, can be stored in a single location.
        %   Additionally, each value in the map stores the current state of
        %   that object, i.e. collision and pose, if applicable, given the
        %   current robot configuration and other scene objects. Whenever
        %   the overall state of the scene (joint configuration, set of
        %   collision objects, pose of collision objects) is updated, the
        %   state in the map is also updated.
        SceneObjectsMap

        %RigidBodyKeyMap A map that relates the rigid body names to the their scene objects map keys
        %   Because many of the rigidBodyTree-specific algorithms use rigid
        %   body names as function inputs and outputs, but most of the app
        %   uses scene object keys to identify anything in the scene, it is
        %   often necessary to look up the scene object key (UUID) from a
        %   rigid body name. This map provides that functionality.
        RigidBodyKeysMap

        %CurrentConfig The current configuration of the robot
        CurrentConfig

        %CurrentConfigKey Key associated with the current configuration
        %   When the current configuration corresponds to a stored
        %   configuration in the configurations model, this value is a
        %   non-empty string. When the configuration is not stored, this
        %   value is empty. Whereas the scene and solver models only show
        %   show the present state, the configurations model stores sets of
        %   model states (i.e. joint configuration, collision status, and
        %   solver details) that can be loaded and unloaded, and these
        %   states are identified by their key.
        CurrentConfigKey
    end

    methods
        function obj = CoreSceneState
            %CoreSceneState Constructor

            obj.RigidBodyTree = [];
            obj.SceneObjectsMap = containers.Map.empty;
            obj.RigidBodyKeysMap = containers.Map.empty;
            obj.CurrentConfig = [];
            obj.CurrentConfigKey = string.empty;
        end

        function setup(obj, robot)
            %setup Set up data

            % Store a copy of the rigidBodyTree object
            internalRobot = copy(robot);
            internalRobot.DataFormat = 'column';
            obj.RigidBodyTree = internalRobot;

            obj.resetData()
        end

        function setupFromModel(obj, model)
            %setupFromFile Set up the object using an AppModels object
            %   Given a robotics.ikdesigner.internal.model.AppModels
            %   object, load the data into the current app model.

            obj.RigidBodyTree = model.RigidBodyTree;
            obj.SceneObjectsMap = model.SceneObjectsMap;
            obj.RigidBodyKeysMap = model.RigidBodyKeysMap;
            obj.CurrentConfig = model.CurrentConfig;
            obj.CurrentConfigKey = model.CurrentConfigKey;

        end

        function initialize(~)

        end
    end

    methods (Access = private)
        function resetData(obj)
            %resetData Reset data members to default states
            
            obj.SceneObjectsMap = containers.Map.empty;
            obj.RigidBodyKeysMap = containers.Map.empty;
            obj.CurrentConfig = obj.RigidBodyTree.homeConfiguration;
            obj.CurrentConfigKey = string.empty;
        end
    end

end