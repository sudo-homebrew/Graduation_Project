classdef SceneBrowserView < robotics.ikdesigner.internal.view.View
%This class is for internal use only. It may be removed in the future.

%   SceneBrowserView View that lets users browse the robot and objects in the scene

%   Copyright 2021 The MathWorks, Inc.

    events
        %ObjectSelectionChange Event to indicate that a new object has been clicked on
        ObjectSelectionChange

        %ObjectDeleteRequest Request an object be deleted
        ObjectDeleteRequest

        %RequestMarkerBodyChange Event to request that a given marker body be set as the marker body
        RequestMarkerBodyChange
    end

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        %Parent Handle to parent UIFigure
        Parent

        %BrowserTree Handle to the uitree that makes up the view
        BrowserTree

        %SceneObjectsMap Map of the objects in the scene
        SceneObjectsMap

        RobotNode

        SceneNode
    end

    properties (Constant)
        INCOLLISIONICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'collision_16.png');
        COLLISIONFREEICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'noCollision_16.png');
    end

    properties (Constant)
        ROBOTNODELABEL = string(message('robotics:ikdesigner:scenebrowser:RobotNodeLabel'))

        SCENENODELABEL = string(message('robotics:ikdesigner:scenebrowser:SceneNodeLabel'))

        ASSIGNMARKERMENUTEXT = string(message('robotics:ikdesigner:scenebrowser:AssignMarkerMenuText'))

        DELETEMENUTEXT = string(message('robotics:ikdesigner:scenebrowser:DeleteMenuText'))
    end

    methods
        function obj=SceneBrowserView(fig)
        %SceneBrowserView Constructor

        % Create a grid and initialize a browser view
            obj.Parent = fig;
            gridLayout = uigridlayout(obj.Parent, 'ColumnWidth', {'1x'}, 'Rowheight', {'1x'});
            obj.BrowserTree = uitree(gridLayout, 'tree', 'SelectionChangedFcn', @(evt, src)obj.notifyObjectHighlightSelection(evt));
            obj.SceneObjectsMap = containers.Map.empty;
        end

        function setup(obj, robot, modelSceneObjectsMap, rigidBodyKeysMap)
        %setup Set up the scene browser view

        % Remove all current nodes from the tree
            while ~isempty(obj.BrowserTree.Children)
                waitfor(obj.BrowserTree.Children(1), 'BeingDeleted', 'off');
                delete(obj.BrowserTree.Children(1));
            end

            % Reset the scene objects map
            obj.SceneObjectsMap = containers.Map.empty;

            % The basic tree has two nodes, one for the robot and one for the scene
            obj.RobotNode = uitreenode(obj.BrowserTree,'Text',obj.ROBOTNODELABEL,'NodeData',struct('Key', []));
            obj.SceneNode = uitreenode(obj.BrowserTree,'Text',obj.SCENENODELABEL,'NodeData',struct('Key', []));

            % Some of the nodes derive from rigid body nodes, so it is
            % necessary to create those first
            bodies = [{robot.Base} robot.Bodies];
            for i = 1:numel(bodies)
                bodyKey = rigidBodyKeysMap(bodies{i}.Name);
                obj.addSceneObjectToView(bodyKey, bodies{i}.Name, bodyKey);
            end

            % Add the remaining objects to the uitree & scene objects map
            remainingObjKeys = setdiff(modelSceneObjectsMap.keys, obj.SceneObjectsMap.keys);
            for i = 1:numel(remainingObjKeys)
                obj.addSceneObjectToView(remainingObjKeys{i}, modelSceneObjectsMap(remainingObjKeys{i}).Name, modelSceneObjectsMap(remainingObjKeys{i}).RigidBodyKey);
            end
        end

        function initialize(obj)
        %initialize Initialize the scene browser view

            expand(obj.BrowserTree);

        end

        function updateSceneContent(obj, sceneModelChangedEvent)
        %updateSceneContent Add or remove objects from the scene
        %   Upon notification that the scene contents have changed,
        %   this method compares the keys of the Scene Browser's scene
        %   objects map with those of the model. If there are any
        %   differences, the scene browser's map is updated to ensure
        %   they are synced. These updates in turn update the visuals
        %   in the canvas.

            modelSceneObjMap = sceneModelChangedEvent.SceneObjectsMap;

            % Check for any deleted collision objects and remove them
            delObjectKeys = setdiff(obj.SceneObjectsMap.keys, modelSceneObjMap.keys);
            for i = 1:numel(delObjectKeys)
                obj.removeSceneObjectFromView(delObjectKeys{i});
            end

            % Check for newly added bodies and add them
            newObjectKeys = setdiff(modelSceneObjMap.keys, obj.SceneObjectsMap.keys);
            for i = 1:numel(newObjectKeys)
                modelMapData = modelSceneObjMap(newObjectKeys{i});
                obj.addSceneObjectToView(newObjectKeys{i}, modelMapData.Name, modelMapData.RigidBodyKey);
            end
        end

        function updateSelection(obj, selectionEvent)
        %updateSelection Update the selected node

            obj.BrowserTree.SelectedNodes = obj.SceneObjectsMap(selectionEvent.SceneObjectKey).Node;

        end

        function updateCollisionState(obj, collisionEventData)
        %updateCollisionState Update collision state of objects in the scene
        %   Objects in collision have a red X associated with their
        %   node display. Those not in collision are shown in green.

            if collisionEventData.RBTCollisionState == robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated
                % If collision hasn't been evaluated for this
                % configuration, it should be cleared
                obj.resetCollisionState;
                return;
            end

            modelSceneObjectsMap = collisionEventData.SceneObjectsMap;
            objKeys = modelSceneObjectsMap.keys;

            for i = 1:modelSceneObjectsMap.Count
                objNode = obj.SceneObjectsMap(objKeys{i}).Node;

                % Choose what to display
                if (objNode.Parent ~= obj.RobotNode) && (objNode.Parent ~= obj.SceneNode)
                    % Collisions are only displayed on top-level objects
                    objNode.Icon = '';
                elseif isempty(modelSceneObjectsMap(objKeys{i}).State)
                    objNode.Icon = obj.COLLISIONFREEICON;
                else
                    objNode.Icon = obj.INCOLLISIONICON;
                end
            end
        end
    end

    methods (Access = private)

        function resetCollisionState(obj)
        %resetCollisionState Reset the collision state of the browser tree
        %   This method ensures that all the node icons are reset to
        %   their default empty state (meaning that no collision icon
        %   is displayed).

            objKeys = obj.SceneObjectsMap.keys;
            for i = 1:obj.SceneObjectsMap.Count
                objNode = obj.SceneObjectsMap(objKeys{i}).Node;
                objNode.Icon = '';
            end
        end

        function removeSceneObjectFromView(obj, key)
        %removeSceneObjectFromView Remove the item and visualization from the view
        %   Remove the item with the specified key from the scene
        %   objects map and remove the associated node from the uitree.

            objNode = obj.SceneObjectsMap(key).Node;
            delete(objNode);
            obj.SceneObjectsMap.remove(key);
        end

        function addSceneObjectToView(obj, key, objName, bodyKey)
        %addSceneObjectToView Add new objects to the view
        %   Given a key from the scene model's scene objects map and a
        %   corresponding handle, this method adds the visualization of
        %   that object the scene canvas and updates the corresponding
        %   scene objects map using the same key. The keys of all scene
        %   objects maps always match, but this one stores only
        %   visualization data (patches and info needed for callbacks).
        %   The method also assigns patch callbacks to ensure correct
        %   interactive behavior in the canvas.

            if ~isempty(bodyKey)
                % This object is part of the rigid body
                if strcmp(key, bodyKey)
                    % This object is the body and we have to add the node
                    newNode = uitreenode(obj.RobotNode,'Text',objName,'NodeData',struct('Key', key));
                    newNode.ContextMenu = obj.createRigidBodyContextMenu(key);
                else
                    % This object is associated with a body but differs
                    bodyNode = obj.SceneObjectsMap(bodyKey).Node;
                    newNode = uitreenode(bodyNode,'Text',objName,'NodeData',struct('Key', key));
                end
            else
                % This object is something in the scene
                newNode = uitreenode(obj.SceneNode,'Text',objName,'NodeData',struct('Key', key));

                % Add a context menu, which allows this to be deleted
                newNode.ContextMenu = obj.createSceneContextMenu(key);
            end

            % Register the object in the scene objects map
            obj.SceneObjectsMap(key) = struct('Node', newNode, 'RigidBodyKey', bodyKey);
        end

        function notifyObjectHighlightSelection(obj, selectionEvent)
        %notifyObjectHighlightSelection Callback when a user clicks on an object

        % Only allow the user to select one node at a time
            firstSelectedNodeIdx = 1;
            objKey = selectionEvent.SelectedNodes(firstSelectedNodeIdx).NodeData.Key;

            % If the user selects the robot or scene nodes, do nothing
            if isempty(objKey)
                return;
            end

            % Send this data in an event
            event = robotics.ikdesigner.internal.event.ViewSelectionEventData(objKey, []);
            notify(obj, "ObjectSelectionChange", event);
        end
    end

    methods
        function notifyObjectDeleteRequest(obj, nodeKey)
        %notifyObjectDeleteRequest Send an event to indicate that marker body deletion has been requested

        % Send the event along with the key of the body to be deleted
            event = robotics.ikdesigner.internal.event.DeleteRequestEvent(nodeKey);
            notify(obj, "ObjectDeleteRequest", event);
        end

        function notifyMarkerPoseBodyChangeRequest(obj, nodeKey)
        %notifyMarkerPoseBodyChangeRequest Send an event to indicate that marker pose body should be changed
        %   This method is called when the marker pose body is changed
        %   to a new body with key given by NODEKEY.

        % Send the event along with the key of the body to which the
        % marker will be assigned.
            event = robotics.ikdesigner.internal.event.ViewSelectionEventData(nodeKey, nodeKey);
            notify(obj, "RequestMarkerBodyChange", event);

        end

        function contextMenuHandle = createSceneContextMenu(obj, nodeKey)
        %createSceneContextMenu Create a context menu for items in the scene
        %   This method creates a context menu containing one entry,
        %   "Delete", and assigns a callback to that menu item. This
        %   method is called by non-rigidbody objects in the scene
        %   browser, so the callbacks can be specific to an individual
        %   node.

            contextMenuHandle = uicontextmenu(obj.Parent);
            deleteMenu = uimenu(contextMenuHandle, 'Text', obj.DELETEMENUTEXT);
            deleteMenu.MenuSelectedFcn = @(src, evt)obj.notifyObjectDeleteRequest(nodeKey);
        end

        function contextMenuHandle = createRigidBodyContextMenu(obj, nodeKey)
        %createSceneContextMenu Create a context menu for rigid bodies
        %   This method creates a context menu for rigid bodies in the
        %   scene browser. This context menu allows users to select the
        %   body as the pose marker body.

            contextMenuHandle = uicontextmenu(obj.Parent);
            menuItem = uimenu(contextMenuHandle, 'Text', obj.ASSIGNMARKERMENUTEXT);
            menuItem.MenuSelectedFcn = @(src, evt)obj.notifyMarkerPoseBodyChangeRequest(nodeKey);
        end
    end
end
