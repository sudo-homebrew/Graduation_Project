classdef FigureStateHandler < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %FigureStateHandler Class to control state of figure in the Scene Canvas.

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)

        %Axes Handle to the figure axes
        Axes

        %BodyDisplayArray Cell array of items used for body visualization
        %   The fast visualization uses a cell array of objects to update
        %   the correct patches. See RigidBodyTreeVisualizationHelper for
        %   details.
        BodyDisplayArray

        %RBTLineData Cell array of lines in the rigid body tree for fast visualization
        %   The fast visualization uses a cell array of line data to update
        %   the correct lines (which connect the frames). See
        %   RigidBodyTreeVisualizationHelper for details.
        RBTLineData

        %BaselineTTree Baseline transform tree
        %   The fast visualization relies on a baseline transform tree
        %   defined when the tree is first visualized (all other transforms
        %   are relative). See the RigidBodyTreeVisualizationHelper for
        %   more details.
        BaselineTTree

        %HGTransforms Cell array of HG Transforms for fast visualization
        HGTransforms

        %FigureManager Handle to the figure manager, which defines figure interactions
        FigureManager

        %NumBodies Number of bodies in the rigid body tree object
        NumBodies

        %ShowTag Unique identifier of the rigid body tree associated with this visualization
        ShowTag

        %SceneObjectsMap Containers.Map object containing visualization data
        %   The scene objects map is a containers.Map objects where the
        %   keys are unique identifiers and the values are structures
        %   containing visualization-specific data. The keys of the scene
        %   objects map are synced with a scene objects map in the model,
        %   and other views have equivalent maps, but the values contain
        %   data that is specific to the view.
        SceneObjectsMap

        %PatchKeysInHighlight String array of scene objects map keys for which the associated patches are highlighted
        PatchKeysInHighlight = string.empty
    end

    properties (Access = private)
        %NotifyBodySelectedCB Callback to notify body selection
        NotifyBodySelectedCB
    end

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)
        %BodyHighlightHistoryMap Containers.Map containing the recent history of body edge higlights by color
        %   This property is a containers.Map object that relates a
        %   highlight color to the item that was previously highlighted in
        %   that color. The map keys are strings that represent the color
        %   in question. The map's values are row vectors with up to 2
        %   elements that indicate the key of the selected object in the
        %   scene objects map and, if applicable, the scene objects map key
        %   of the rigid body associated with the object. Therefore, for a
        %   given color, the map will always provide a way to determine the
        %   last scene object that was highlighted in that color. This is
        %   particularly useful when a body is highlighted in a color, and
        %   then subsequently highlighted in another color. When the second
        %   color is removed, the goal is to return to the previous color.
        %   This map can be used to check for instances where such a return
        %   is necessary.
        BodyHighlightHistoryMap
    end

    methods
        function obj = FigureStateHandler(modelSceneObjectsMap, rigidBodyKeysMap, tree, ax, notifySelectedCallback, displayCollisions)
            %FigureStateHandler Constructor

            if nargin < 6
                displayCollisions = false;
            end

            obj.Axes = ax;
            obj.NotifyBodySelectedCB = notifySelectedCallback;

            % Initialize the axes and then apply a hold so all objects are
            % displayed
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.resetScene(tree.TreeInternal, obj.Axes);
            hold(obj.Axes, 'all');
            set(obj.Axes, 'Clipping', 'off');
            obj.Axes.Position = [0 0 1 1];

            % Initialize the figure state handler's map of objects in the
            % scene. This adds the objects to the map but does not yet add
            % the visualization; that is handled in the show method
            obj.initializeSceneObjectsMap(modelSceneObjectsMap);

            % Use the home configuration as the baseline
            baselineConfig = tree.homeConfiguration;

            % Display the rigid body tree. This adds the tree to the canvas
            % view, displays all the associated patch objects, and records
            % them in the scene objects map.
            obj.initializeShow(tree, rigidBodyKeysMap, baselineConfig, displayCollisions);

            % Add HG Transforms to the figure and set baseline Ttree
            obj.BaselineTTree = tree.TreeInternal.forwardKinematics(baselineConfig);
            obj.HGTransforms = robotics.manip.internal.RigidBodyTreeVisualizationHelper.addHGTransforms(obj.BodyDisplayArray, obj.Axes);
            obj.RBTLineData = obj.BodyDisplayArray(:,3:4);
            obj.ShowTag = tree.ShowTag;
            obj.NumBodies = tree.NumBodies;

            % Initialize containers.Map objects
            obj.BodyHighlightHistoryMap = containers.Map;
        end

        function updateAxesConfig(obj, ttree)
            %updateAxesConfig Update the configuration of the tree on the figure axes

            % Update the figure
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.fastVisualizationUpdate(obj.HGTransforms, obj.RBTLineData, ttree, obj.BaselineTTree);
        end

        function clearObjectSelection(obj, colorsToClear)
            %clearObjectSelection Reset the edge color of objects of given colors

            for i = 1:size(colorsToClear,1)
                obj.FigureManager.clearPatchHighlights(colorsToClear(i,:));
            end

        end

        function removeHighlightColor(obj, colorToRemove)
            %removeHighlightColor Clear objects highlighted with this color and remove highlight history

            obj.clearObjectSelection(colorToRemove);
            colorHistoryKey = obj.getSelectionHistoryKey(colorToRemove);
            if obj.BodyHighlightHistoryMap.isKey(colorHistoryKey)
                obj.BodyHighlightHistoryMap.remove(colorHistoryKey);
            end

        end

        function selectObject(obj, selectionEvent, selectionColor)
            %selectObject Select the object specified by the event
            %   Highlight the object specified in the event data. Given a
            %   key that corresponds to an object in the scene, this method
            %   will highlight the associated visual patches. If the object
            %   is associated to a rigid body, a joint axis is drawn.

            % Clear existing edge selections in the selection color and
            % reset the highlight to last value if applicable
            obj.resetPatchHighlights(selectionColor);
            obj.applyEdgeHighlightToBody(selectionColor, selectionEvent.SceneObjectKey, selectionEvent.AssocRigidBodyKey);

        end
        
        function updateSceneContent(obj, sceneModelChangedEvent)
            %updateSceneContent Add or remove objects from the scene
            %   Upon notification that the scene contents have changed,
            %   this method compares the keys of the Figure State Handler's
            %   scene objects map with those of the model. If there are any
            %   differences, the figure state handler's map is updated to
            %   ensure they are synced. These updates in turn update the
            %   visuals in the canvas.

            newSceneObjMap = sceneModelChangedEvent.SceneObjectsMap;
            
            % Check for any deleted collision objects and remove them
            delObjectKeys = setdiff(obj.SceneObjectsMap.keys, newSceneObjMap.keys);
            for i = 1:numel(delObjectKeys)
                obj.removeObjectsFromCanvas(delObjectKeys{i});
            end

            % Check for newly added bodies and add them
            newObjectKeys = setdiff(newSceneObjMap.keys, obj.SceneObjectsMap.keys);
            for i = 1:numel(newObjectKeys)
                obj.addObjectsToCanvas(newObjectKeys{i}, newSceneObjMap(newObjectKeys{i}).Handle);
            end
        end

        function highlightBodyFace(obj, objectKey, faceColor)
            %highlightBodyFace Change the face color of a body
            %   This method changes the face color of a given patch to a
            %   value specified by FACECOLOR. To ensure it can be changed
            %   back, the method updates the OriginalFacecolor field of the
            %   value struct in the Scene Objects Map.

            % Get the body patches
            patchData = obj.SceneObjectsMap(objectKey);
            bodyPatches = patchData.PatchHandles;

            % Store the original color of the body patches
            numPatches = numel(bodyPatches);
            patchData.OriginalFaceColor = cell(1, numPatches);
            patchData.OriginalColorData = cell(1, numPatches);
            for i = 1:numPatches
                % Some patches don't have a single uniform color; instead
                % they use a "flat" FaceColor and assign colors by
                % vertex/face using the data in the FaceVertexCData. It is
                % necessary to support this format because the axes patch
                % objects make use of it. 
                patchData.OriginalFaceColor{i} = bodyPatches(i).FaceColor;
                patchData.OriginalColorData{i} = bodyPatches(i).FaceVertexCData ;
            end
            obj.SceneObjectsMap(objectKey) = patchData;

            % Highlight bodies by changing the face color
            obj.PatchKeysInHighlight = [obj.PatchKeysInHighlight string(objectKey)];
            for i = 1:length(bodyPatches)
                bodyPatches(i).FaceColor = faceColor;
            end

        end

        function clearBodyFaceHighlights(obj)
            %clearBodyFaceHighlights Reset color of highlighted bodies to their original values

            for i = 1:numel(obj.PatchKeysInHighlight)
                patchData = obj.SceneObjectsMap(obj.PatchKeysInHighlight(i));
                for j = 1:numel(patchData.PatchHandles)
                    set(patchData.PatchHandles(j), ...
                        'FaceColor', patchData.OriginalFaceColor{j}, ...
                        'FaceVertexCData', patchData.OriginalColorData{j});
                end
            end

            obj.PatchKeysInHighlight = string.empty;
        end
    end

    methods (Access = private)

        function initializeSceneObjectsMap(obj, modelSceneObjectsMap)
            %initializeSceneObjectsMap Initialize the scene objects map using the data from the corresponding map in the scene model
            %   This method initializes the figure state handler's scene
            %   objects map by adding all the objects in the scene model's
            %   map to the map with the corresponding keys. However, no
            %   objects from the rigid body tree are visualized, as this
            %   visualization is handled independently during the initial
            %   show method call.

            obj.SceneObjectsMap = containers.Map.empty;

            % Iterate over all the keys in the source map and add these to
            % the local map, storing the handle and leaving room for the
            % patch handles.
            sourceKeys = modelSceneObjectsMap.keys;
            for i = 1:numel(sourceKeys)
                objHandle = modelSceneObjectsMap(sourceKeys{i}).Handle;

                if isa(objHandle, 'rigidBody') || isa(objHandle, 'rigidBodyJoint')
                    % Rigid body tree visualization is handled in the
                    % initial show method to ensure compatibility with the
                    % fast show method.
                    obj.addObjectsToCanvas(sourceKeys{i}, objHandle, []);
                else
                    obj.addObjectsToCanvas(sourceKeys{i}, objHandle);
                end

                % Update the body index, which is set for rigid body
                % objects and is unset (-1) for all others
                sceneObjMapEntry = obj.SceneObjectsMap(sourceKeys{i});
                if isa(objHandle, 'rigidBody')
                    sceneObjMapEntry.BodyIndex = objHandle.BodyInternal.Index;
                else
                    sceneObjMapEntry.BodyIndex = -1;
                end
                obj.SceneObjectsMap(sourceKeys{i}) = sceneObjMapEntry;
            end

        end

        function initializeShow(obj, tree, bodyKeysMap, baselineConfig, displayCollisions)
            %initializeShow Plot the robot and store figure data

            % Assign conditionals
            basePosition = zeros(1,3);
            baseRotation = 0;
            preserveExistingFigure = true;
            displayVisuals = 'on';
            displayFrames = 'on';

            % Configure the figure manager so that there are no banner
            % widgets, and the user can click in white space without
            % resetting the overall figure selection
            initBannerWidgets = false;
            enableClearInfoOnMousePress = false;
            figManagerOpts = robotics.manip.internal.createFigureManagerOpts(initBannerWidgets, enableClearInfoOnMousePress);

            [~, obj.BodyDisplayArray, obj.FigureManager] = simpleShow(tree.TreeInternal, baselineConfig, obj.Axes.Parent, ...
                displayCollisions, [basePosition baseRotation], preserveExistingFigure, displayVisuals, displayFrames, figManagerOpts);

            % Now that the patches have been added to the scene, assign
            % them to the associated values in the scene objects map and
            % assign the proper callbacks. 
            for bodyIdx = 1:size(obj.BodyDisplayArray,1)
                [bodyKey, objHandle, patchHandles] = obj.getBodyVizDetailsFromIndex(tree, bodyKeysMap, bodyIdx);
                obj.assignDataAndCallbacksToSceneObjMapEntry(bodyKey, objHandle, patchHandles);
            end

            % Hold so that objects besides the robot can be plotted
            hold(obj.Axes, 'all');
        end

        function jointAxisPatch = drawJointAxis(obj, bodyPatches, bodyHandle)
            %drawJointAxis Draw joint axes for bodies
            %   This method draws the yellow joint axis when a body is
            %   highlighted using the selection tool. The method is only
            %   called when the selected object is directly associated
            %   with a rigid body.

            vizBodyIdx = obj.getBodyVisualizationCellArrayIdx(bodyHandle);
            bodyHGTForm = bodyPatches(1).Parent;

            % The HG transform associated with the patches will be used to
            % move the joint axis, which means that the axis must be drawn
            % at the point it would be at when the transform was
            % initialized, i.e. the pose for the baseline transform tree.
            bodyPatchTransform = obj.BaselineTTree{vizBodyIdx};

            % Try to draw the joint axis. If the joint is fixed, no axis
            % will be drawn. Otherwise it is drawn and assigned an HG
            % transform that correctly positions it in space.
            jointAxisPatch = robotics.manip.internal.RigidBodyTreeVisualizationHelper.drawJointAxisMemoryLess(obj.Axes, bodyHandle.BodyInternal, bodyPatchTransform);
            if ~isempty(jointAxisPatch)
                jointAxisPatch.Parent = bodyHGTForm;
            end
        end

        function vizBodyIdx = getBodyVisualizationCellArrayIdx(obj, bodyHandle)
            %getBodyVisualizationCellArrayIdx Get the row index of a particular body in the cell array used for fast visualization
            %   Internally, the rigidBodyTree uses an index that places the
            %   base at zero and all other bodies after it. However, the
            %   fast visualization tools rely on a cell array where the
            %   first N rows correspond to the bodies, and the (N+1)th row
            %   corresponds to the base. This method extracts the index
            %   needed for use with the fast visualization cell array given
            %   a rigid body object handle.

            bodyIdx = bodyHandle.BodyInternal.Index;
            if bodyIdx > 0
                vizBodyIdx = bodyIdx;
            else
                vizBodyIdx = obj.NumBodies;
            end
        end

        function [bodyKey, objHandle, patchHandles] = getBodyVizDetailsFromIndex(obj, tree, bodyKeysMap, bodyIdx)
            %getBodyVizDetailsFroIndex Assign callbacks to rigid body patches
            %   Assign selection-callbacks to patches that are used to
            %   visualize the rigid body tree. When these patches are
            %   clicked on, the associated selection callbacks will fire.
            %   This method primarily ensures that the correct data will be
            %   passed to the callback, but it also assigns the patch
            %   handles from the visualization to the corresponding spot in
            %   the scene objects map value structure.
            
                bodyMeshArray = obj.BodyDisplayArray{bodyIdx,1};
                frameMeshPatchHandle = obj.BodyDisplayArray{bodyIdx,2};

                % Get the correct key using the body name
                if bodyIdx == tree.NumBodies+1
                    bodyKey = bodyKeysMap(tree.Base.Name);
                    objHandle = tree.Base;
                else
                    bodyKey = bodyKeysMap(tree.Bodies{bodyIdx}.Name);
                    objHandle = tree.Bodies{bodyIdx};
                end

                % Assign the patch handles to the scene object struct and
                % update the scene objects map
                patchHandles = [bodyMeshArray{:} frameMeshPatchHandle];
        end

        function assignPatchCallbacks(obj, patchObj, bodyKey)
            %assignPatchCallbacks Assign callbacks to each patch object
            %   Specify patch callbacks. When these are set to empty, it
            %   overwrites any previous setting and is equivalent to them
            %   not being set at all.

            % The button down function specifies what happens when the
            % object is clicked on
            patchObj.ButtonDownFcn = @(src,evt)obj.notifyObjectHighlightSelection(bodyKey);

            % Undo previous callbacks that would link to nonexistent
            % objects by setting the context menu and delete function
            % callbacks to empty
            patchObj.UIContextMenu = [];
            patchObj.DeleteFcn = [];

        end

        function notifyObjectHighlightSelection(obj, bodyKey)
            %notifyObjectHighlightSelection Callback when a user clicks on an object

            obj.NotifyBodySelectedCB(bodyKey);
        end

        function addObjectsToCanvas(obj, key, objHandle, patchHandles)
            %addObjectsToCanvas Add new objects to the figure canvas
            %   Given a key from the scene model's scene objects map and a
            %   corresponding handle, this method adds the visualization of
            %   that object the scene canvas and updates the corresponding
            %   scene objects map using the same key. The keys of all scene
            %   objects maps always match, but this one stores only
            %   visualization data (patches and info needed for callbacks).
            %   The method also assigns patch callbacks to ensure correct
            %   interactive behavior in the canvas.
            
            if nargin < 4
                % If no patch handles are provided, use the show method to
                % calculate them and turn off edges by default
                [~, patchHandles] = objHandle.show('Parent', obj.Axes);
                for i = 1:numel(patchHandles)
                    patchHandles(i).LineStyle = 'none';
                end
            end

            obj.assignDataAndCallbacksToSceneObjMapEntry(key, objHandle, patchHandles);
        end

        function assignDataAndCallbacksToSceneObjMapEntry(obj, key, objHandle, patchHandles)
            %assignDataToSceneObjMapEntry Assign values to a new or existing scene object map entry
            %   This method assigns the object handle and patch handle to a
            %   scene objects map entry with a given key. It also assigns
            %   the callbacks to that patch object.

            if ~obj.SceneObjectsMap.isKey(key)
                % If the structure hasn't been created yet, initialize it
                sceneObjEntry = struct();
            else
                sceneObjEntry = obj.SceneObjectsMap(key);
            end

            sceneObjEntry.Handle = objHandle;
            sceneObjEntry.PatchHandles = patchHandles;

            % Update the scene objects map
            obj.SceneObjectsMap(key) =  sceneObjEntry;

            % Assign the patches to the canvas
            for i = 1:numel(patchHandles)
                obj.assignPatchCallbacks(patchHandles(i), key);
            end

        end

        function removeObjectsFromCanvas(obj, key)
            %removeObjectsFromCanvas Remove objects and their visualizations from the figure canvas
            %   This method removes objects from the canvas given their
            %   key. It also resets any associated states or derived
            %   objects such as transforms.

            % Get the associated patch handles and remove them from the
            % visualization
            patchHandles = obj.SceneObjectsMap(key).PatchHandles;
            for i = 1:numel(patchHandles)
                delete(patchHandles(i));
            end

            % Remove the key from the highlighted set if applicable
            obj.PatchKeysInHighlight(strcmp(obj.PatchKeysInHighlight, key)) = [];

            % Remove the key/value from the map
            obj.SceneObjectsMap.remove(key);
            
        end

        function setPatchHighlightHistory(obj, selectionColor, sceneObjKey, assocRigidBodyKey)
            %setPatchHighlightHistory Assign patch highlight history to a given edge color

            bodyHighlightKey = obj.getSelectionHistoryKey(selectionColor);
            obj.BodyHighlightHistoryMap(bodyHighlightKey) = [sceneObjKey assocRigidBodyKey];

        end

        function resetPatchHighlights(obj, deselectionColor)
            %resetPatchHighlights Reset patch edge highlights to previous selection color
            %   A patch can be highlighted with an edge color, and each
            %   edge color can highlight patches for at most one body at a
            %   time. However, some patches might be highlighted in one
            %   color, and then another color "on top" of that, meaning
            %   that when the patch highlighting is reset, the highlight
            %   should go back to the the "bottom" color. This method is
            %   used to reset to the last used color. 

            % Clear highlights for all patches of a given edge color
            obj.FigureManager.clearPatchHighlights(deselectionColor);

            % Get the body key / assoc rigid body key that was highlighted
            % in this color. In the case where no bodies were previously
            % highlighted in this color (e.g. when a color highlight is
            % first introduced), exit, as no further action is needed.
            deselectedObjKey = obj.getSelectionHistoryKey(deselectionColor);
            if ~obj.BodyHighlightHistoryMap.isKey(deselectedObjKey)
                return;
            end
            deslectedBodyKeys = obj.BodyHighlightHistoryMap(deselectedObjKey);

            % Reset the values for the selection color being cleared
            obj.BodyHighlightHistoryMap(deselectedObjKey) = string.empty;

            % Check if this key is also highlighted in a secondary color
            highlightColors = obj.BodyHighlightHistoryMap.keys;
            bodyKeysInHighlight = obj.BodyHighlightHistoryMap.values;
            secondHighlightIdx = cellfun(@(s)(~isempty(s) && all(contains(s,deslectedBodyKeys))), bodyKeysInHighlight, 'UniformOutput', true);

            % If the body key is highlighted in another color, reset to
            % that color rather than clearing highlight altogether
            if any(secondHighlightIdx)
                secondHighlightColorKey = highlightColors{secondHighlightIdx};
                secondHighlightColor = obj.getSelectionColorFromKey(secondHighlightColorKey);
                if numel(deslectedBodyKeys) < 2
                    assocDeselectedBodyKey = string.empty;
                else
                    assocDeselectedBodyKey = deslectedBodyKeys(2);
                end
                obj.applyEdgeHighlightToBody(secondHighlightColor, deslectedBodyKeys(1), assocDeselectedBodyKey);
            end
        end

        function applyEdgeHighlightToBody(obj, selectionColor, sceneObjKey, assocRigidBodyKey)
            %applyEdgeHighlightToBody Apply an edge highlight to a body in a given color

            sceneObjKey = string(sceneObjKey);
            assocRigidBodyKey = string(assocRigidBodyKey);
            
            objHandle = obj.SceneObjectsMap(sceneObjKey).Handle;
            bodyPatches = obj.SceneObjectsMap(sceneObjKey).PatchHandles;
            selectionColorKey = obj.getSelectionHistoryKey(selectionColor);
            obj.BodyHighlightHistoryMap(selectionColorKey) = sceneObjKey;

            % If the selected object doesn't have any associated visuals,
            % select the corresponding body instead
            if isempty(bodyPatches)
                bodyPatches = obj.SceneObjectsMap(assocRigidBodyKey).PatchHandles;
                objHandle = obj.SceneObjectsMap(assocRigidBodyKey).Handle;
            end

            if isa(objHandle, 'rigidBody') || isa(objHandle, 'rigidBodyJoint')
                obj.drawJointAxis(bodyPatches, objHandle);
            end

            % Highlight bodies
            for i = 1:length(bodyPatches)
                bodyPatches(i).LineStyle = '-';
                bodyPatches(i).EdgeColor = selectionColor;
            end

            % Update highlight history for this color
            obj.setPatchHighlightHistory(selectionColor, sceneObjKey, assocRigidBodyKey);
        end
    end

    methods (Static, Access = private)
        function selectionKey = getSelectionHistoryKey(selectionColor)
            %getSelectionHistoryKey Map color to key without loss of precision

            selectionKey = sprintf("[%0.16f %0.16f %0.16f]", selectionColor(1), selectionColor(2), selectionColor(3));
        end

        function selectionColor = getSelectionColorFromKey(selectionKey)
            selectionColor = str2num(selectionKey); %#ok<ST2NM> 
        end
    end
end
