classdef InteractiveVisualizationHelper < robotics.manip.internal.RigidBodyTreeVisualizationHelper
%This class is for internal use only. It may be removed in the future.

%INTERACTIVEVISUALIZATIONHELPER Visualization helper for interactiveRigidBodyTree
%   This class acts as an interface between the figure axes, the marker
%   objects, and the interactiveRBT class.

%   Copyright 2019-2021 The MathWorks, Inc.

    properties (GetAccess = ?robotics.manip.internal.InternalAccess, SetAccess = private)

        %HGTransforms - Cell array of HG Transforms
        HGTransforms

        %RBTLineData - (N+1)x2 Cell array of line data
        %   The RBTLineData cell array has N+1 rows (for the N rigid bodies
        %   and base in the associated RBT) and 2 columns. The first column
        %   is a cell array of line handles, and the second column is a
        %   cell array of child body indices. For the ith row, the cell
        %   array in the first column contains all the lines that start at
        %   the rigidBody with index K. The second column contains the
        %   indices of all the child bodies to which these lines connect.
        %   For the base, the index N+1 is used. This data is used when
        %   lines are updated during visualization updates.
        RBTLineData

        %BaselineConfig - Configuration of the rigidBodyTree when it is first plotted and hgTransforms are initialized
        BaselineConfig

        %Marker - Marker object
        Marker

        %IKMarkerCallbackHandles - IK Marker callback handles
        IKMarkerCallbackHandles

        %JtMarkerCallbackHandles - Joint Control Marker callback handles
        JtMarkerCallbackHandles

        %Properties that are unchanged (not state)

        %AxesHandle - Handle to the figure axes
        AxesHandle

        %InteractiveRBTBackend - Handle to the interactiveRBT backend object
        InteractiveRBTBackend
    end

    properties (Dependent, SetAccess = private)

        %MarkerPose - Pose of the marker
        MarkerPose
    end

    methods
        function obj = InteractiveVisualizationHelper(backendObj)
        %InteractiveVisualizationHelper Constructor

        % Handle to the interactiveRBT backend
            obj.InteractiveRBTBackend = backendObj;

            % Get the configuration
            config = backendObj.Configuration;

            % Initialize figure with tree and initialize callback handles
            figHandle = figure('Visible', 'on', ...
                               'Name', 'Interactive Visualization', ...
                               'CloseRequestFcn',@hideFigure);
            obj.AxesHandle = newplot(figHandle);
            [~, axesObjects] = show(backendObj.RigidBodyTree.TreeInternal, config, 'Frames', backendObj.Frames, 'Parent', obj.AxesHandle);

            % Assign UI Menus to bodies
            obj.assignUIMenusToBodies(axesObjects);

            % Add HG Transforms to the figure and set baseline Ttree
            obj.BaselineConfig = backendObj.RigidBodyTree.TreeInternal.forwardKinematics(config);
            obj.HGTransforms = obj.addHGTransforms(axesObjects, obj.AxesHandle);
            obj.RBTLineData = axesObjects(:,3:4);

            % Initialize figure view
            hold(obj.AxesHandle, 'on')
            grid(obj.AxesHandle, 'on');
            rotate3d(obj.AxesHandle, 'off');

            % Add the marker
            if backendObj.ShowMarker
                obj.createMarker;
            end
        end

        function delete(obj)
        %delete Destructor
        %   The destructor ensures the figure behaves as expected after
        %   the interactiveRigidBodyTree object has been deleted.
        %   Specifically, it changes the close behavior to close rather
        %   than hide the figure, deletes any hidden figures, and
        %   removes any interactive elements -- markers and context
        %   menus -- from open figures, and renames open figures to
        %   clarify that the interactive elements have been deleted.

        % If the figure has already been deleted (e.g. by a call to
        % "force close all"), then the figure handle will no longer
        % exist and none of the subsequent steps are necessary
            if isempty(obj.AxesHandle) || ~isvalid(obj.AxesHandle)
                return;
            end

            % Reset the close request of the associated figure so that the
            % figure can be deleted
            obj.AxesHandle.Parent.CloseRequestFcn = 'closereq';

            % Remove interactive markers
            if ~isempty(obj.Marker)
                delete(obj.Marker);
            end

            % Remove the figure callbacks
            obj.removeUIContextMenus();

            % If the figure handle is still valid but the figure is hidden,
            % delete the figure; otherwise (if the figure is visible),
            % rename the figure but leave it open
            if strcmp(obj.AxesHandle.Parent.Visible, 'off') && ishandle(obj.AxesHandle.Parent)
                close(obj.AxesHandle.Parent);
            else
                obj.AxesHandle.Parent.Name = sprintf('%s [%s]', obj.AxesHandle.Parent.Name, message('robotics:robotmanip:interactiverigidbodytree:DeletedFigureMessage').getString);
            end
        end
    end

    methods (Access = ?robotics.manip.internal.InternalAccess)
        function createMarker(obj)
        %createMarker Create marker in the figure axes
        %   This marker creates a marker that lines up with the body to
        %   which it has been assigned. The type of marker is selected
        %   based on the value of the MarkerControlMethod property.
        %   Each new marker replaces and old marker, which is first
        %   deleted. The button callbacks are stored on the marker
        %   objects themselves.

        % Extract properties from the backend model
            bodyPose = obj.InteractiveRBTBackend.MarkerBodyPose;
            controlMethod = obj.InteractiveRBTBackend.MarkerControlMethod;
            scaleFactor = obj.InteractiveRBTBackend.MarkerScaleFactor;

            % If a marker already exists, delete it (this will delete the
            % figure objects)
            if ~isempty(obj.Marker)
                delete(obj.Marker);
            end

            % Select the marker based on the value of MarkerControlMethod
            if strcmp(controlMethod, 'InverseKinematics')
                obj.Marker = robotics.manip.internal.IKControlMarker(obj.AxesHandle, bodyPose, scaleFactor, 'ExternalButtonMoveCB', @(tgtPose)obj.triggerMarkerMoveActions(tgtPose));
            else
                % When control method is 'JointControl', the marker will
                % have different appearance depending on the joint type and
                % axis, which are stored on the jointBody. The jointBody is
                % also needed to map the joint to its index in the
                % rigidBodyTree Configuration.
                markerBodyName = obj.InteractiveRBTBackend.MarkerBodyName;
                if strcmp(obj.InteractiveRBTBackend.RigidBodyTree.Base.Name, markerBodyName)
                    % If the selected body is the base, joint control
                    % cannot be applied, so the marker is instead turned
                    % off and a warning is thrown.
                    warning(message('robotics:robotmanip:interactiverigidbodytree:BaseNoJointControl', markerBodyName));
                    obj.InteractiveRBTBackend.ShowMarker = false;
                else
                    jointBody = obj.InteractiveRBTBackend.RigidBodyTree.getBody(markerBodyName);
                    if strcmp(jointBody.Joint.Type, 'fixed')
                        % If the body has a fixed joint, the joint control
                        % marker doesn't make sense, so the marker is instead
                        % turned off and a warning is thrown.
                        warning(message('robotics:robotmanip:interactiverigidbodytree:FixedBodyNoJointControl', markerBodyName));
                        obj.InteractiveRBTBackend.ShowMarker = false;
                    else
                        obj.Marker = robotics.manip.internal.JtControlMarker(obj.AxesHandle, bodyPose, jointBody, obj.InteractiveRBTBackend, obj.HGTransforms, obj.RBTLineData, obj.BaselineConfig, scaleFactor);
                    end
                end
            end
        end

        function updateIK(obj, targetPose)
        %updateIK Update inverse kinematics given a target pose
        %   This method is just a wrapper for the backend where it is
        %   called.

            obj.InteractiveRBTBackend.updateIK(obj.Configuration, targetPose);
        end

        function updateAxesConfig(obj)
        %updateAxesConfig Update the configuration of the tree on the figure axes

            % Get the TTree
            Ttree = obj.InteractiveRBTBackend.RigidBodyTree.TreeInternal.forwardKinematics(obj.InteractiveRBTBackend.Configuration);

            % Update the figure
            obj.fastVisualizationUpdate(obj.HGTransforms, obj.RBTLineData, Ttree, obj.BaselineConfig);
        end

        function updateMarker(obj, markerPose)
        %updateMarker Update the marker given a pose
        %   This updates the marker pose, which is defined relative to
        %   the figure (and therefore rbt) origin.

            obj.Marker.updateMarker(markerPose);
        end

        function showFigure(obj)
        %showFigure Make figure visible and put it in focus

        % Enable visibility
            obj.AxesHandle.Parent.Visible = 'on';

            % Bring into focus
            figure(obj.AxesHandle.Parent);
            % Reset the nextplot behavior
            obj.AxesHandle.Parent.NextPlot = 'add';

            % Bring into focus
            figure(obj.AxesHandle.Parent);
        end

        function toggleMarkerOnOff(obj)

            if obj.InteractiveRBTBackend.ShowMarker
                delete(obj.Marker);
                obj.InteractiveRBTBackend.ShowMarker = false;
            else
                % Update marker pose
                obj.InteractiveRBTBackend.ShowMarker = true;
                obj.createMarker;
            end

        end

        function toggleMarkerControlMethod(obj)

            if strcmp(obj.InteractiveRBTBackend.MarkerControlMethod, 'InverseKinematics')
                delete(obj.Marker);
                obj.InteractiveRBTBackend.MarkerControlMethod = 'JointControl';
                obj.InteractiveRBTBackend.ShowMarker = true;
                obj.createMarker;
            else
                delete(obj.Marker);
                obj.InteractiveRBTBackend.MarkerControlMethod = 'InverseKinematics';
                obj.InteractiveRBTBackend.ShowMarker = true;
                obj.createMarker;

                % Update the pose target
                obj.InteractiveRBTBackend.updatePoseTarget;
            end

        end

        function assignMarkerToBodyByName(obj, bodyName)

        % Make ShowMarker visible and assign the body name to the
        % MarkerBodyName property
            obj.InteractiveRBTBackend.ShowMarker = true;
            obj.InteractiveRBTBackend.MarkerBodyName = bodyName;

            % Create the marker, which will replace any existing marker
            obj.createMarker;

            % Update the pose target
            obj.InteractiveRBTBackend.updatePoseTarget;
        end

        function cm = addInteractiveContextMenu(obj, cm, bodyName)
        %getBodyContextMenu Helper to set up body-attached context
        %   menu

        % Create child menu items for this figure
            uimenu(cm, 'Label', 'Set body as marker body', 'Callback', {@(src, evt)assignMarkerToBodyByName(obj, bodyName)});
            uimenu(cm, 'Label', 'Toggle marker on/off', 'Callback', {@(src, evt)toggleMarkerOnOff(obj)}, 'Separator', 'on');
            uimenu(cm, 'Label', 'Toggle marker control method', 'Callback', {@(src, evt)toggleMarkerControlMethod(obj)});
        end

        function assignUIMenusToBodies(obj, bodyDisplayObjArray)

            bodyNameArray = [obj.InteractiveRBTBackend.RigidBodyTree.BodyNames obj.InteractiveRBTBackend.RigidBodyTree.BaseName];

            for bodyIndex = 1:size(bodyDisplayObjArray,1)
                % For each rigidBody link, there is one associated
                % hgtransform object

                % Cell arrays of patches for each visual
                visualPatchArray = bodyDisplayObjArray{bodyIndex,1};
                for j = 1:length(visualPatchArray)
                    currentUIContextMenu = visualPatchArray{j}.UIContextMenu;
                    visualPatchArray{j}.UIContextMenu = obj.addInteractiveContextMenu(currentUIContextMenu, bodyNameArray{bodyIndex});
                end

                % Patch objects for each frame
                framePatch = bodyDisplayObjArray{bodyIndex,2};
                if ~isempty(framePatch)
                    currentUIContextMenu2 = framePatch.UIContextMenu;
                    framePatch.UIContextMenu = obj.addInteractiveContextMenu(currentUIContextMenu2, bodyNameArray{bodyIndex});
                end
            end
        end


        function removeUIContextMenus(obj)

            for bodyIndex = 1:size(obj.HGTransforms,1)
                % For each rigidBody link, there is one associated
                % hgtransform object
                if ~isvalid(obj.HGTransforms{bodyIndex})
                    continue;
                end

                % Cell arrays of patches for each visual
                visualPatchArray = obj.HGTransforms{bodyIndex}.Children;
                for j = 1:length(visualPatchArray)
                    currentUIContextMenuEntries = visualPatchArray(j).UIContextMenu.Children;
                    for k = 1:(length(currentUIContextMenuEntries)-1)
                        currentUIContextMenuEntries(k).Enable = matlab.lang.OnOffSwitchState('Off');
                        currentUIContextMenuEntries(k).Visible = matlab.lang.OnOffSwitchState('Off');
                    end
                end
            end
        end
    end

    %% IK update methods used by the interactive markers
    methods (Access = ?robotics.manip.internal.InteractiveMarker)
        function triggerMarkerMoveActions(obj, targetPose)

            obj.InteractiveRBTBackend.updateIK(obj.InteractiveRBTBackend.Configuration, targetPose);
            obj.updateAxesConfig;
        end
    end

    %% Get/Set methods for properties used for data transfer

    methods

        function markerPose = get.MarkerPose(obj)
        %get.MarkerPose Get method for markerPose
        %   The marker pose is stored on the marker object with which
        %   it is associated.

            if ~isempty(obj.Marker) && isvalid(obj.Marker)
                markerPose = obj.Marker.MarkerPose;
            else
                markerPose = [];
            end

        end
    end
end

function hideFigure(src, ~)
%hideFigure Hide figure callback function
%   This function is used to override the close request on a figure.
%   When the figure is closed via the UI or the "close" function, the
%   visibility is instead set to OFF. This ensures that the figure can
%   later be restored.

%hideFigure hide instead of delete the figure
    src.Visible = 'off';

    % While the figure is hidden, any calls to figures from sources other
    % than this object should create a new figure
    src.NextPlot = 'new';
end
