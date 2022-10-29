classdef ConstraintsBrowserView < robotics.ikdesigner.internal.view.View
%This function is for internal use only. It may be removed in the future.

%   ConstraintsBrowserView View that lets users browse the list of current constraints

%   Copyright 2021-2022 The MathWorks, Inc.

    events
        %NodeSelectionChange Indicate that the node selection has changed
        NodeSelectionChange

        %ConstraintDeleteRequest Request that a given constraint be deleted
        ConstraintDeleteRequest

        %ConstraintEditRequest Request that a given constraint be edited
        ConstraintEditRequest

        %RequestConstraintStateChange Request to enable/disable constraint
        RequestConstraintStateChange

        %RequestMarkerDisplayToggle Request to enable/disable marker
        RequestMarkerDisplayToggle
    end

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        %Parent Handle to parent UIFigure
        Parent

        %BrowserTree Handle to the uitree that makes up the view
        BrowserTree

        %ConstraintsMap Map of the constraints
        ConstraintsMap

        %PresetConstraintsNode Parent node for the marker constraint
        PresetConstraintsNode

        %UserDefinedConstraintsNode Parent node for user-defined constraints
        UserDefinedConstraintsNode

        %MarkerPoseTargetNode Node used to display the marker target pose
        MarkerPoseTargetNode
    end

    properties (Constant)
        %MarkerPoseTargetKey Key associated with the marker pose constraint
        %   The marker pose constraint is stored in the constraints map,
        %   but it used a reserved key that cannot overlap with a UUID
        %   value. Since this constraint cannot be removed, the value is
        %   persistent. This allows the key to be easily accessed from a
        %   constraints map in the model or views.
        MarkerPoseTargetKey = robotics.ikdesigner.internal.constants.Data.MARKERPOSETARGETKEY
    end

    properties (Constant)
        CONSTRAINTMETICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'constraintMet_16.png');
        CONSTRAINTUNMETICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'constraintUnmet_16.png');
    end

    properties (Constant, Access = private)
        PRESETCONSTRAINTSLABEL = string(message('robotics:ikdesigner:constraintsbrowser:PresetConstraintsNodeLabel'))

        USERDEFINEDCONSTRAINTSLABEL = string(message('robotics:ikdesigner:constraintsbrowser:CustomConstraintsNodeLabel'))

        TOGGLEMARKERMENUENTRY = string(message('robotics:ikdesigner:constraintsbrowser:UIMenuToggleMarkerEntry'))

        EDITMENUENTRY = string(message('robotics:ikdesigner:constraintsbrowser:UIMenuEditEntry'))

        DELETEMENUENTRY = string(message('robotics:ikdesigner:constraintsbrowser:UIMenuDeleteEntry'))

        HELPLINKLABEL = string(message('robotics:ikdesigner:constraintsbrowser:HelpLinkLabel'))
    end

    methods
        function obj = ConstraintsBrowserView(fig)
        %ConstraintsBrowserView Constructor

        % Create a grid and initialize a browser view
            obj.Parent = fig;
            gridLayout = uigridlayout(obj.Parent, 'ColumnWidth', {'1x'}, 'Rowheight', {'1x', 'fit'});
            obj.BrowserTree = uitree(gridLayout, 'checkbox', ...
                                     'SelectionChangedFcn', @(evt,src)obj.notifyObjectHighlightSelection(evt), ...
                                     'CheckedNodesChangedFcn', @(evt,src)obj.notifyObjectToggleRequest(src));
            obj.ConstraintsMap = containers.Map.empty;

            % Add a link at the base to link to doc
            helpLabel = uihyperlink(gridLayout, "Text", obj.HELPLINKLABEL);
            helpLabel.Layout.Row = 2;
            helpLabel.HyperlinkClickedFcn = @(~,~)helpview(fullfile(docroot,'robotics','helptargets.map'),'AddAndResolveConstraintsExample');

        end

        function setup(obj, modelConstraintsMap)
        %setup Set up the view given model constraint data

        % Remove all current nodes from the tree
            while ~isempty(obj.BrowserTree.Children)
                waitfor(obj.BrowserTree.Children(1), 'BeingDeleted', 'off');
                delete(obj.BrowserTree.Children(1));
            end

            % Reset the scene objects map
            obj.ConstraintsMap = containers.Map.empty;

            % The marker pose target node is visualized differently from
            % the other constraint nodes, which are under a separate parent
            % node
            obj.addMarkerPoseTargetNode(modelConstraintsMap);

            % Add constraints to the view
            remainingObjKeys = setdiff(modelConstraintsMap.keys, obj.ConstraintsMap.keys);
            for i = 1:numel(remainingObjKeys)
                % By default, all constraints are enabled. Ensure the new
                % node state matches the model: unset or enabled implies
                % the node is enabled, while disabled means the node is
                % also disabled.
                isEnabled = robotics.ikdesigner.internal.model.ConstraintState.checkIfEnabled(modelConstraintsMap(remainingObjKeys{i}).State);

                obj.addConstraintToView(remainingObjKeys{i}, modelConstraintsMap(remainingObjKeys{i}).Name, ...
                                        modelConstraintsMap(remainingObjKeys{i}).Type, isEnabled);
            end
        end

        function initialize(obj)
        %initialize Initialize the view for a new session

            expand(obj.BrowserTree);
        end

        function updateSelection(obj, selectionEvent)
        %updateSelection Update the selected node

            obj.BrowserTree.SelectedNodes = obj.ConstraintsMap(selectionEvent.ConstraintKey).Node;

        end

        function updateViewContent(obj, constraintEventData)
        %updateViewContent Update the view given changes to the constraint state or values
        %   The event data includes an update type, which is an
        %   enumeration indicating the kind of update. This method
        %   routes the event to the proper method based on the update
        %   type. For more info on what the enumerations mean, refer to
        %   the original enum definition file.

            switch constraintEventData.UpdateType
              case robotics.ikdesigner.internal.model.ConstraintUpdate.StateReset
                obj.resetConstraintState;

              case robotics.ikdesigner.internal.model.ConstraintUpdate.MapValuesChange
                obj.updateConstraintsMapValues(constraintEventData.ConstraintsMap, constraintEventData.ChangedKeys)

              case robotics.ikdesigner.internal.model.ConstraintUpdate.StateChange
                obj.updateConstraintState(constraintEventData.ConstraintsMap);
            end
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function updateConstraintsMapValues(obj, modelConstraintsMap, keysToUpdate)
        %updateConstraintsMapValues Add or remove objects from the constraints map
        %   This method checks for changes to the model constraints map
        %   (added or removed keys) and updates the view constraints
        %   map to correspond.

            % Check for any deleted collision objects and remove them
            delObjectKeys = setdiff(obj.ConstraintsMap.keys, modelConstraintsMap.keys);
            for i = 1:numel(delObjectKeys)
                obj.removeConstraintFromView(delObjectKeys{i});
            end

            % Check for newly added bodies and add them
            newObjectKeys = setdiff(modelConstraintsMap.keys, obj.ConstraintsMap.keys);
            for i = 1:numel(newObjectKeys)
                modelMapData = modelConstraintsMap(newObjectKeys{i});
                isNodeEnabled = true; % New nodes are enabled by default
                obj.addConstraintToView(newObjectKeys{i}, modelMapData.Name, modelMapData.Type, isNodeEnabled);
            end

            % Check for any changed keys and update them
            changedExistingKeys = intersect(keysToUpdate, obj.ConstraintsMap.keys);
            for i = 1:numel(changedExistingKeys)
                modelMapData = modelConstraintsMap(changedExistingKeys{i});
                obj.editConstraintInView(changedExistingKeys{i}, modelMapData.Name, modelMapData.Type);
            end

        end
    end

    methods (Access = private)
        function resetConstraintState(obj)
        %resetConstraintState Set the constraint view to its default when the state is unknown
        %   This removes associated icons that are used to indicate
        %   pass/fail.

            objKeys = obj.ConstraintsMap.keys;
            for i = 1:obj.ConstraintsMap.Count
                constraintNode = obj.ConstraintsMap(objKeys{i}).Node;
                constraintNode.Icon = '';
            end

        end

        function updateConstraintState(obj, modelConstraintsMap)
        %updateCollisionState Update collision state of objects in the scene
        %   Objects in collision have a red X associated with their
        %   node display. Those not in collision are shown in green.

            constraintKeys = modelConstraintsMap.keys;

            for i = 1:modelConstraintsMap.Count
                objNode = obj.ConstraintsMap(constraintKeys{i}).Node;

                % The constraint state is just the violations output.
                % Violations equal to zero are passing, but a numerical
                % tolerance must be applied.
                if modelConstraintsMap(constraintKeys{i}).State == robotics.ikdesigner.internal.model.ConstraintState.Disabled || ...
                        modelConstraintsMap(constraintKeys{i}).State == robotics.ikdesigner.internal.model.ConstraintState.Unset
                    objNode.Icon = '';
                elseif modelConstraintsMap(constraintKeys{i}).State == robotics.ikdesigner.internal.model.ConstraintState.Pass
                    objNode.Icon = obj.CONSTRAINTMETICON;
                else
                    objNode.Icon = obj.CONSTRAINTUNMETICON;
                end
            end
        end

        function newNode = addConstraintToView(obj, constraintKey, objName, constraintType, isNodeEnabled)
        %addConstraintToView Add new objects to the view

        % If this is the first constraint node, create the parent
            if obj.ConstraintsMap.Count == 1
                obj.UserDefinedConstraintsNode = uitreenode(obj.BrowserTree,'Text',obj.USERDEFINEDCONSTRAINTSLABEL,'NodeData',struct('Key', []));
                obj.BrowserTree.expand;
            end

            % Create a node for the constraint
            constraintTypeName = robotics.ikdesigner.internal.toolstrip.ConstraintType.getUserFacingName(constraintType);
            constraintNodeName = sprintf("%s [%s]", objName, constraintTypeName);
            newNode = uitreenode(obj.UserDefinedConstraintsNode,'Text', constraintNodeName, 'NodeData',struct('Key', constraintKey));

            % Enable the node, or leave it disabled (the default for new
            % nodes in a checkbox tree)
            if isNodeEnabled
                obj.checkNode(newNode);
            end

            % Add a context menu, which allows this to be modified or
            % removed
            newNode.ContextMenu = obj.createConstraintContextMenu(constraintKey);

            % Register the object in the scene objects map
            obj.ConstraintsMap(constraintKey) = struct('Node', newNode);
        end

        function editConstraintInView(obj, constraintKey, objName, constraintType)
        %editConstraintInView Modify existing displayed objects in the view

            nodeToEdit = obj.ConstraintsMap(constraintKey).Node;
            constraintTypeName = robotics.ikdesigner.internal.toolstrip.ConstraintType.getUserFacingName(constraintType);
            constraintNodeName = sprintf("%s [%s]", objName, constraintTypeName);
            nodeToEdit.Text = constraintNodeName;
        end

        function addMarkerPoseTargetNode(obj, modelConstraintsMap)
        %addMarkerPoseTargetNode Add a node in the uitree for the marker pose target

            constraintKey = obj.MarkerPoseTargetKey;
            markerPoseTargetEntry = modelConstraintsMap(constraintKey);

            % Get the displayed constraint name
            constraintName = markerPoseTargetEntry.Name;
            constraintType = markerPoseTargetEntry.Type;
            constraintTypeName = robotics.ikdesigner.internal.toolstrip.ConstraintType.getUserFacingName(constraintType);
            constraintNodeName = sprintf("%s [%s]", constraintName, constraintTypeName);

            % Initialize the node and assign the context menu
            obj.PresetConstraintsNode = uitreenode(obj.BrowserTree,'Text', obj.PRESETCONSTRAINTSLABEL, 'NodeData',struct('Key', []));
            obj.MarkerPoseTargetNode = uitreenode(obj.PresetConstraintsNode,'Text', constraintNodeName, 'NodeData',struct('Key', constraintKey));
            obj.MarkerPoseTargetNode.ContextMenu = obj.createMarkerPoseContextMenu(constraintKey);

            % Register the object in the scene objects map
            obj.ConstraintsMap(constraintKey) = struct('Node', obj.MarkerPoseTargetNode);

            % When a new node is added, it is unchecked by default. Enable
            % the constraint as long as it is not disabled in the model.
            isEnabled = robotics.ikdesigner.internal.model.ConstraintState.checkIfEnabled(markerPoseTargetEntry.State);
            if isEnabled
                obj.checkNode(obj.MarkerPoseTargetNode);
            end
        end

        function removeConstraintFromView(obj, key)
        %removeSceneObjectFromView Remove the item and visualization from the view
        %   Remove the item with the specified key from the scene
        %   objects map and remove the associated node from the uitree.

            objNode = obj.ConstraintsMap(key).Node;
            delete(objNode);
            obj.ConstraintsMap.remove(key);

            % When no user-defined constraints remain, remove the parent
            % node
            if obj.ConstraintsMap.Count == 1
                obj.UserDefinedConstraintsNode.delete;
            end
        end

        function notifyObjectHighlightSelection(obj, selectionEvent)
        %notifyObjectHighlightSelection Notify the controller that the user has selected a node in the browser tree

        % Only allow the user to select one node at a time
            firstSelectedNodeIdx = 1;
            objKey = selectionEvent.SelectedNodes(firstSelectedNodeIdx).NodeData.Key;

            % Send this data in an event
            event = robotics.ikdesigner.internal.event.ConstraintSelectionEventData(objKey);
            notify(obj, "NodeSelectionChange", event);
        end

        function notifyObjectEditRequest(obj, nodeKey)
        %notifyObjectEditRequest Notify the controller that the user has requested to edit a constraint

        % Update the view so that is also selects this item, then
        % notify the controller
            obj.BrowserTree.SelectedNodes = obj.ConstraintsMap(nodeKey).Node;
            event = robotics.ikdesigner.internal.event.ConstraintSelectionEventData(nodeKey);
            notify(obj, "NodeSelectionChange", event);

            % Notify the request to edit the selected constraint
            notify(obj, "ConstraintEditRequest");
        end

        function notifyObjectDeleteRequest(obj, nodeKey)
        %notifyObjectDeleteRequest Notify the controller that the user has deleted a constraint

        % Send this data in an event
            event = robotics.ikdesigner.internal.event.DeleteRequestEvent(nodeKey);
            notify(obj, "ConstraintDeleteRequest", event);
        end

        function notifyMarkerToggleRequest(obj)
        %notifyMarkerToggleRequest Notify the controller that the marker display should be toggled

            notify(obj, "RequestMarkerDisplayToggle");

        end

        function contextMenuHandle = createMarkerPoseContextMenu(obj, nodeKey)
        %createContextMenu Create a context menu
        %   This method creates a context menu containing one entry,
        %   "Delete", and assigns a callback to that menu item. This
        %   method is called by objects in the scene browser, so the
        %   callbacks can be specific to an individual node.

            contextMenuHandle = uicontextmenu(obj.Parent);

            hideMarkerMenu = uimenu(contextMenuHandle, 'Text', obj.TOGGLEMARKERMENUENTRY);
            hideMarkerMenu.MenuSelectedFcn = @(src, evt)obj.notifyMarkerToggleRequest();

            % Add the edit option
            editMenu = uimenu(contextMenuHandle, 'Text', obj.EDITMENUENTRY);
            editMenu.MenuSelectedFcn = @(src, evt)obj.notifyObjectEditRequest(nodeKey);
        end

        function contextMenuHandle = createConstraintContextMenu(obj, nodeKey)
        %createConstraintContextMenu Create a context menu
        %   This method creates a context menu containing one entry,
        %   "Delete", and assigns a callback to that menu item. This
        %   method is called by objects in the scene browser, so the
        %   callbacks can be specific to an individual node.

            contextMenuHandle = uicontextmenu(obj.Parent);

            % Add the edit option
            editMenu = uimenu(contextMenuHandle, 'Text', obj.EDITMENUENTRY);
            editMenu.MenuSelectedFcn = @(src, evt)obj.notifyObjectEditRequest(nodeKey);

            % Add the delete option
            deleteMenu = uimenu(contextMenuHandle, 'Text', obj.DELETEMENUENTRY);
            deleteMenu.MenuSelectedFcn = @(src, evt)obj.notifyObjectDeleteRequest(nodeKey);
        end

        function notifyObjectToggleRequest(obj, src)
            %notifyObjectToggleRequest Enable/disable constraints in model

            %Determine the nodes that were enabled and disabled
            enabledNodes = setdiff(src.LeafCheckedNodes, src.PreviousLeafCheckedNodes);
            disabledNodes = setdiff(src.PreviousLeafCheckedNodes, src.LeafCheckedNodes);

            enabledConstraintKeys = strings(size(enabledNodes));
            disabledConstraintKeys = strings(size(disabledNodes));
            for i = 1:numel(enabledNodes)
                enabledConstraintKeys(i) = enabledNodes(i).NodeData.Key;
            end
            for j = 1:numel(disabledNodes)
                disabledConstraintKeys(j) = disabledNodes(j).NodeData.Key;
            end

            constraintStateData = robotics.ikdesigner.internal.event.ConstraintToggleRequestData(enabledConstraintKeys, disabledConstraintKeys);
            notify(obj, "RequestConstraintStateChange", constraintStateData);

        end

        function checkNode(obj, nodeToEnable)
            %checkNode Select the checkbox beside a node in the browser tree

            obj.BrowserTree.CheckedNodes = [obj.BrowserTree.CheckedNodes; nodeToEnable];
        end
    end
end
