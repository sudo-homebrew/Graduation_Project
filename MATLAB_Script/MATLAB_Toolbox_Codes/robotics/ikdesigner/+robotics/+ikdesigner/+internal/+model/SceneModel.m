classdef SceneModel < robotics.ikdesigner.internal.model.Model
    %This class is for internal use only and may be removed in a future release

    %SceneModel Model containing data related to scene contents and current state (joint config and collision)

    %   Copyright 2021 The MathWorks, Inc.

    events

        %CollisionStateAdded Event to indicate that the collision state was updated for some configuration
        CollisionStateAdded

        %CollisionStateChanged Event to indicate that the collision state was updated for the current configuration
        CollisionStateChanged

        %ConstraintBodySelectionChanged Event to indicate that a body selection associated with a constraint was updated
        ConstraintBodySelectionChanged

        %ConstraintSceneDisplayUpdated Event to indicate that a scene canvas visual aid associated with a constraint has been updated
        ConstraintSceneDisplayUpdated

        %MarkerBodyUpdated Event to indicate that the marker is associated with a new body
        MarkerBodyUpdated
        
        %RBTStateChanged Event to indicate a change in robot configuration state
        RBTStateChanged

        %SceneContentsChanged Event to indicate an object has been added or removed from the scene
        SceneContentsChanged

        %SceneSelectionChanged Event to indicate a change in scene object selection
        SceneSelectionChanged
    end

    properties (Access = private)        
        %DefaultCollisionState Default overall collision state should be un-evaluated
        DefaultCollisionState = robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated;
    end

    properties (Constant)
        %DEFAULTIGNORESELFCOLLISIONS Default state for self collisions
        DEFAULTIGNORESELFCOLLISIONS = "off";
    end

    properties (Dependent)

        %RigidBodyTree RigidBodyTree object
        RigidBodyTree

        %SceneObjectsMap A map containing all the objects in the scene
        SceneObjectsMap

        %RigidBodyKeyMap A map that relates the rigid body names to the keys of the rigid bodies in the corresponding scene objects map
        RigidBodyKeyMap

        %Config The current robot joint configuration
        Config

        %ConfigKey
        ConfigKey
    end

    properties (SetAccess = ?matlab.unittest.TestCase)

        %SharedModelState Handle to the model of shared scene states
        SharedModelState

        %SelectedSceneObjKey Key for the object that is currently selected in the scene
        SelectedSceneObjKey

        %PrevConfig The previous robot joint configuration
        PrevConfig

        %IgnoreSelfCollisions Boolean indicating whether or not to ignore self collision checks
        IgnoreSelfCollisions

        %CollisionRawData
        CollisionRawData

        %CollisionState An enumeration indicating the current overall collision state
        CollisionState

        %CollisionObjectKeys A cell array of all the keys that correspond to non-RigidBodyTree collision objects
        CollisionObjectKeys = {}

        %MarkerBodyKey The key of the body the marker is associated with
        MarkerBodyKey
    end

    properties (Dependent)
        %TransformTree A cell array of transforms corresponding to the output of treeInternal.forwardKinematics()
        TransformTree

        %MarkerBodyName The name of the rigid body associated with the pose marker
        MarkerBodyName
    end

    methods

        function obj = SceneModel(sharedModelState)
            %SceneModel Constructor

            obj.SharedModelState = sharedModelState;

            % Maps have to be explicitly initialized to empty
            obj.SceneObjectsMap = containers.Map.empty;

        end

        function setup(obj, modelData)
            %setup Reset data members and set up with new incoming data

            if nargin > 1
                % Populate data from existing model
                obj.CollisionObjectKeys = modelData.CollisionObjectKeys;
                obj.CollisionState = modelData.CollisionState;
            else
                % Initialize the list of objects in the scene objects map
                obj.addRobotToSceneObjectsMap();
                obj.CollisionObjectKeys = {};
                obj.CollisionState = obj.DefaultCollisionState;
            end

            % In the current approach, the self collision check is always
            % reset to the default state
            obj.IgnoreSelfCollisions = obj.DEFAULTIGNORESELFCOLLISIONS;
        end

        function initialize(obj)
            %initialize Initialize a new session

            % Initialize state properties that are dependent on the rigid
            % body tree object. Use the last body in the tree as the
            % default marker body and selected body.
            obj.Config = obj.RigidBodyTree.homeConfiguration;
            obj.PrevConfig = obj.Config;
            obj.MarkerBodyKey = obj.RigidBodyKeyMap(obj.RigidBodyTree.BodyNames{end});
            obj.SelectedSceneObjKey = obj.RigidBodyKeyMap(obj.RigidBodyTree.BodyNames{end});
        end
    end

    methods
        function updateConfig(obj, config)
            %updateConfig Update rigid body tree state pertaining to configuration
            %   Update the current configuration and any derived states,
            %   e.g. the previous configuration. Additionally, the method
            %   fires an event to indicate the model state has been
            %   updated.

            obj.PrevConfig = obj.Config;
            obj.Config = config;

            % Clear the configuration key label
            obj.ConfigKey = string.empty;

            % Trigger an eventdata to indicate the state has been updated.
            % The event includes the new configuration and the robot's
            % complete associated transform tree.
            evtData = robotics.ikdesigner.internal.event.RBTStateEventData(...
                obj.Config, obj.TransformTree);
            notify(obj, 'RBTStateChanged', evtData);

            % Reset collision
            obj.resetCollisionState;
            obj.notifyCollisionStateChange();
        end

        function updateSceneSelection(obj, viewSelectionEvtData)
            %updateSceneSelection Change the selected object in the scene model
            %   Update the value of the selected object in the scene. The
            %   selected item can be a body, joint, or collision object, as
            %   indicated in the eventdata.

            % The scene objects are stored in the scene objects map and
            % identified using a unique key
            obj.SelectedSceneObjKey = viewSelectionEvtData.SceneObjectKey;

            % Trigger an event to indicated that the selected object state
            % has changed. This event will be used to update the associated
            % views.
            selectionEvent = robotics.ikdesigner.internal.event.ViewSelectionEventData(...
                obj.SelectedSceneObjKey, obj.SceneObjectsMap(obj.SelectedSceneObjKey).RigidBodyKey);
            evtName = 'SceneSelectionChanged';
            
            notify(obj, evtName, selectionEvent);
        end

        function addCollisionObjectsToSceneModel(obj, newDataEvent)
            %addCollisionObjectsToSceneModel Add collision objects to scene model
            %   This method adds imported collision objects to the scene.
            %   The method updates the scene objects map with the name of
            %   the collision object (using the import source as a name
            %   choice) and the value, sets all necessary associated
            %   properties, and notifies the controllers that the scene has
            %   been updated.

            for i = 1:numel(newDataEvent.Data)
                collisionObject = newDataEvent.Data{i};
                collisionObjectName = newDataEvent.DataNames{i};
                key = obj.addObjectToSceneObjectsMap(collisionObject, collisionObjectName);

                % The keys of all objects in the scene objects map that
                % correspond to collision objects are stored in a cell
                % array for easy lookup during collision checking tasks
                obj.CollisionObjectKeys{end+1} = key;
            end

            obj.notifySceneContentsChange();

            % Reset collision
            obj.resetCollisionState;
            obj.notifyCollisionStateChange();
        end

        function removeObjectFromSceneModel(obj, deleteEvent)
            %removeObjectFromSceneModel

            % Remove the object from the scene
            obj.SceneObjectsMap.remove(deleteEvent.Key);

            % Remove the object from the list of collision body keys if
            % applicable
            isKeyInCollisionKeysArray = strcmp([obj.CollisionObjectKeys{:}],deleteEvent.Key);
            obj.CollisionObjectKeys(isKeyInCollisionKeysArray) = [];

            % Notify the views that the scene has changed
            obj.notifySceneContentsChange();

            % Reset collision
            obj.resetCollisionState;
            obj.notifyCollisionStateChange();
        end

        function [overallCollisionState, areBodiesColliding] = checkCurrentCollision(obj)
            %checkCollision Check collision with the current active configuration

            config = obj.Config;
            [overallCollisionState, areBodiesColliding] = obj.checkCollision(config);
            obj.setConfigCollisionState(overallCollisionState, areBodiesColliding, true);
        end

        function [overallCollisionState, areBodiesColliding] = checkCollision(obj, config)
            %checkCollision Check collision with the provided configuration

            % Check collision with all objects in the scene
            collisionObjects = obj.getNonRBTCollisionObjects;
            [isColliding, sepDist] = checkCollision(obj.RigidBodyTree, config, collisionObjects, 'Exhaustive', 'on', 'IgnoreSelfCollision', obj.IgnoreSelfCollisions);

            % Determine which bodies are in collision from the separation
            % distance output. When self collisions are ignored, they are
            % excluded from separation distance. To avoid inconsistency,
            % use the all-inclusive format here, i.e. store collisions as a
            % matrix of size (m+1) x (m+1+w) for the m+1 bodies and base,
            % and the w world collision objects.
            areBodiesColliding = false(size(sepDist,1), size(sepDist,1)+numel(collisionObjects));
            areBodiesColliding(:, (size(areBodiesColliding,2)-size(sepDist,2)+1):size(areBodiesColliding,2)) = isnan(sepDist);

            % Update the model collision state at the current configuration
            overallCollisionStateBoolean = any(isColliding);
            overallCollisionState = robotics.ikdesigner.internal.model.RBTCollisionState(overallCollisionStateBoolean);

        end

        function updateCollisionState(obj, collisionState, areBodiesColliding)
            %updateCollisionState Update the collision state from a pre-saved value

            % No need to store the resultant configuration since this is an
            % update applied by a client of the model
            requiresStorage = false;

            % The model collision state indicates whether or not collision
            % has been calculated
            obj.CollisionState = collisionState;
            if collisionState == robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated
                obj.resetCollisionState();
                obj.notifyCollisionStateChange(requiresStorage);
            else
                % The configuration collision state indicates the detailed
                % collision for this configuration
                isAnyBodyInCollision = any(areBodiesColliding, 'all');
                obj.setConfigCollisionState(isAnyBodyInCollision, areBodiesColliding, requiresStorage);
            end
        end

        function resetCollisionState(obj)
            %resetCollisionState Reset the collision states of all objects in the scene that can be in collision

            allCollisionKeys = [obj.CollisionObjectKeys obj.RigidBodyKeyMap.values];
            for i = 1:numel(allCollisionKeys)
                obj.assignValueToMapStruct(obj.SceneObjectsMap, allCollisionKeys{i}, 'State', {})
            end

            % Reset the collision state and raw data
            obj.CollisionState = obj.DefaultCollisionState;
            obj.CollisionRawData = false(size(obj.CollisionRawData));
        end

        function addModelDataToSceneConstraintVisual(obj, evt)
            %addModelDataToSceneConstraintVisual Add data from the scene model to the visual
            %   The constraint visual requires some information that is
            %   only available in the scene model. This includes the index
            %   of the reference body, for which only a key is provided,
            %   and the transform tree of the robot in its current state.
            %   This method receives an event from the toolstrip with
            %   user-facing content, adds the data available only in the
            %   model, and sends the event back out so that it can be
            %   passed to the scene canvas view.

            % Add the current TFTree to the event
            evt.TFTree = obj.TransformTree;

            notify(obj, 'ConstraintSceneDisplayUpdated', evt);

        end

        function updateSceneMarkerBody(obj, evt)
            %updateSceneMarkerBody Associate the marker with a new body
            %   Given an event that contains a key associated to a rigid
            %   body, this method updates the model with that key and
            %   notifies the controller that the marker has been updated.
            %   The controller may then notify the appropriate dependent
            %   visuals.

            obj.MarkerBodyKey = evt.SceneObjectKey;
            notify(obj, 'MarkerBodyUpdated', evt);
        end

        function enableIgnoreSelfCollision(obj, isEnabled)
            %enableSelfCollisionChecks Ignore or include self collision
            %   Choose whether or not collisions between rigid body tree
            %   bodies should be included in collision checking. When self
            %   collisions are ignored, only collisions between the tree
            %   and the environment are considered to be a collision.

            obj.IgnoreSelfCollisions = string(matlab.lang.OnOffSwitchState(isEnabled));

            % This change resets the current and stored collision states

            % Notify the views that the scene has changed
            obj.resetCollisionState;
            obj.notifyCollisionStateChange();
            obj.notifySceneContentsChange();
        end
    end

    % Getter & setter methods
    methods
        function ttree = get.TransformTree(obj)
            %get.TransformTree Returns a cell array of transforms for each body relative to the base
            %   The transform tree is a tool for efficiently computing the
            %   poses of all the bodies in the tree at once without
            %   redundant passes. 

            ttree = obj.RigidBodyTree.TreeInternal.forwardKinematics(obj.Config);
        end

        function mbname = get.MarkerBodyName(obj)
            %get.MarkerBodyName Get the marker body name
            %   The model only stores the marker body key, but for some
            %   tools, it is necessary to get the name. This property
            %   ensures that can happen. 
            
            mbname = obj.SceneObjectsMap(obj.MarkerBodyKey).Name;
        end

        function robot = get.RigidBodyTree(obj)
            %get.RigidBodyTree This value is read-only
            
            robot = obj.SharedModelState.RigidBodyTree;
        end

        function map = get.SceneObjectsMap(obj)
            %get.SceneObjectsMap

            map = obj.SharedModelState.SceneObjectsMap;
        end

        function set.SceneObjectsMap(obj, map)
            %set.SceneObjectsMap

            obj.SharedModelState.SceneObjectsMap = map;
        end

        function map = get.RigidBodyKeyMap(obj)
            %get.RigidBodyKeyMap

            map = obj.SharedModelState.RigidBodyKeysMap;
        end

        function set.RigidBodyKeyMap(obj, map)
            %set.RigidBodyKeyMap

            obj.SharedModelState.RigidBodyKeysMap = map;
        end

        function config = get.Config(obj)
            %get.Config

            config = obj.SharedModelState.CurrentConfig;
        end

        function set.Config(obj, config)
            %set.Config
            
            obj.SharedModelState.CurrentConfig = config;
        end

        function key = get.ConfigKey(obj)
            %get.ConfigKey
            
            key = obj.SharedModelState.CurrentConfigKey;
        end

        function set.ConfigKey(obj, key)
            %set.ConfigKey
            
            obj.SharedModelState.CurrentConfigKey = key;
        end
    end

    methods (Access = private)
        function addRobotToSceneObjectsMap(obj)
            %addRobotToSceneObjectsMap Register the joints and bodies from the rigid body tree with the scene objects map
            %   The Scene objects map is a list of all possible objects in
            %   the scene. It contains the handle to the object, as well as
            %   any necessary descriptors. This function registers the
            %   selectable parts of the robot, i.e. the bodies and joints,
            %   into the scene objects map.

            bodies = [{obj.RigidBodyTree.Base} obj.RigidBodyTree.Bodies];
            for i = 1:numel(bodies)
                bodyKey = obj.addObjectToSceneObjectsMap(bodies{i}, bodies{i}.Name);
                obj.assignValueToMapStruct(obj.SceneObjectsMap, bodyKey, 'RigidBodyKey', bodyKey);

                % A separate map objects maps body names to the
                % corresponding keys. This is useful for efficient
                % interactions with rigidBodyTree methods.
                obj.RigidBodyKeyMap(bodies{i}.Name) = bodyKey;

                % The base does not have a joint, but all other bodies do
                if i > 1
                    obj.addObjectToSceneObjectsMap(bodies{i}.Joint, bodies{i}.Joint.Name, bodyKey);
                end
            end
        end

        function newKey = addObjectToSceneObjectsMap(obj, objHandle, objName, bodyKey)
            %addObjectToSceneObjectsMap Add a new object to the scene

            % Create a new unique key.
            newKey = matlab.lang.internal.uuid;

            % If there is no associated rigid body, leave the body key
            % empty
            if nargin < 4
                bodyKey = [];
            end

            % The value is a structure that contains the object handle,
            % name, and collision state
            obj.SceneObjectsMap(newKey) = struct('Handle', objHandle, ...
                'Name', objName, 'State', [], 'RigidBodyKey', bodyKey);

        end

        function collisionObjCellArray = getNonRBTCollisionObjects(obj)
            %getNonRBTCollisionObjects Get an ordered cell array of all the non-RBT collision bodies

            structArray = obj.SceneObjectsMap.values(obj.CollisionObjectKeys);
            collisionObjCellArray = cellfun(@(x)(x.Handle), structArray, 'UniformOutput',false);
        end

        function setConfigCollisionState(obj, overallCollisionState, areBodiesColliding, requiresStorage)
            %setConfigCollisionState Assign collision states to objects in the scene
            %   This method takes two inputs: the overall collision state,
            %   which is a logical indicating whether any body in the tree
            %   is in collision, and areBodiesColliding, which is a matrix
            %   of logicals that indicates whether any two bodies are in
            %   collision. The method then updates the model's collision
            %   property and the collision states of each of the affected
            %   bodies in the scene objects map. For all the rigid bodies,
            %   the collision state is a cell array of keys of other rigid
            %   bodies and collision objects with which it is in collision.
            %   For all collision objects, the collision state is a cell
            %   array of keys of all the rigid bodies with which it is in
            %   collision (the check collision tools only check for
            %   collision of the tree with itself and collision bodies, not
            %   collision bodies with each other).

            % Reset all collision states to their default (collision-free)
            % state
            obj.resetCollisionState;

            % Update the model collision state
            obj.CollisionState = overallCollisionState;

            % Store the raw collision details, which are used for storing
            % collision data
            obj.CollisionRawData = areBodiesColliding;

            bodyNames = [obj.RigidBodyTree.BodyNames obj.RigidBodyTree.BaseName];

            rigidBodyKeys = obj.RigidBodyKeyMap.values(bodyNames);
            cbKeys = obj.CollisionObjectKeys;

            % The second element of the collision data output is an (M+1)
            % x (M+W+1) matrix indicating whether any two bodies are in
            % collision. The first M+1 indices refer to the M+1 bodies, and
            % the second W indices correspond to the W collision objects.
            % Assign the collision state for the bodies in the tree
            for rbIdx = 1:size(areBodiesColliding, 1)

                % Get the indices of bodies in collision with this body
                rbsInCollision = areBodiesColliding(rbIdx, 1:(obj.RigidBodyTree.NumBodies+1));
                cbsInCollision = areBodiesColliding(rbIdx, (obj.RigidBodyTree.NumBodies+2):end);

                % Get a flat cell array containing all the keys of all objects
                % in collision with this rigid body
                rbKeysInCollision = rigidBodyKeys(rbsInCollision);
                cbKeysInCollision = cbKeys(cbsInCollision);
                allKeysInCollision = [rbKeysInCollision cbKeysInCollision];

                % Assign the list of all collision objects to the
                % corresponding rigid body and its collision bodies
                obj.assignValueToMapStruct(obj.SceneObjectsMap, rigidBodyKeys{rbIdx}, 'State', allKeysInCollision);

                % Add the keys of this body and its corresponding collision
                % body to the state of the collision bodies that it
                % intersects with
                for cbIdx = 1:numel(cbKeysInCollision)
                    cbKey = cbKeysInCollision{cbIdx};
                    cbData = obj.SceneObjectsMap(cbKey);
                    cbData.State = [cbData.State rigidBodyKeys(rbIdx)];
                    obj.SceneObjectsMap(cbKey) = cbData;
                end
            end

            % Notify listeners
            obj.notifyCollisionStateChange(requiresStorage);
        end

        function notifyCollisionStateChange(obj, requiresStorage)
            %notifyCollisionStateChange Notify the controller that collision state has been updated

            collisionData = robotics.ikdesigner.internal.event.CollisionStateData(obj.Config, obj.SceneObjectsMap, obj.CollisionState);
            notify(obj, 'CollisionStateChanged', collisionData);

            obj.notifyModelBecameDirty();
        end

        function notifySceneContentsChange(obj)
            % Collision object is added, removed, or modified
            collisionData = robotics.ikdesigner.internal.event.CollisionStateData(obj.Config, obj.SceneObjectsMap, obj.CollisionState);
            notify(obj, 'SceneContentsChanged', collisionData);

            obj.notifyModelBecameDirty();
        end
    end

    methods (Static, Access = private)
        function assignValueToMapStruct(map, key, name, value)
            %assignValueToMapStruct Assign values to a structure in a containers.Map value
            %   The containers.Map objects only allow first-level indexing
            %   when data is being assigned, meaning that it is not
            %   possible to directly update a structure field inside a
            %   value; the whole structure must be replaced. This is a
            %   helper to expedite that action.

            mapData = map(key);
            mapData.(name) = value;
            map(key) = mapData; %#ok<NASGU>
        end
    end
end