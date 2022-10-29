classdef InspectorViewTemplate < handle
    %This class is for internal use only. It may be removed in the future.

    %   InspectorViewTemplate Inspector view template base class

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = protected)
        %InspectedObjectKey Key of the body currently under inspection
        InspectedObjectKey

    end

    properties (SetAccess = immutable, GetAccess = protected)

        %Parent Parent view
        Parent

        %SendNotificationCB Callback to send a notification via a controller-facing view
        SendNotificationCB

        %Grid Main grid that all other components are attached to
        Grid
    end

    properties (Dependent)
        %Visible Property that controls whether view is shown or not
        Visible
    end

    % Methods that are called by the scene inspector
    methods
        function obj = InspectorViewTemplate(parentObj, parentFig, notificationCallback)
            %InspectorView Constructor

            obj.Parent = parentObj;

            % Add a callback for sending notifications before creating the
            % inspector panels, which may refer to this handle
            obj.SendNotificationCB = notificationCallback;

            % Populate the panel but set default visibility to false
            obj.Grid = obj.createBaseGrid(parentFig);
            obj.createInspectorPanels();
            obj.Visible = false;

            % Start with nothing inspected and setup other data members
            obj.InspectedObjectKey = string.empty;
            obj.setup();
        end

        function populate(obj, objKey, sceneObjMap, sceneModel)
            %populate Populate view with properties and states for a selected body
            %   When the body is first selected, this method populates the
            %   inspector views with properties and states.

            obj.populateSelectionProperties(objKey, sceneObjMap, sceneModel);

        end

        function updateRBTConfiguration(obj, evt)
            %updateCollisionState Update the displayed body pose for the currently selected body

            obj.updateViewGivenNewRBTConfig(evt);
        end

        function updateCollisionState(obj, modelSceneObjectsMap, isCollisionComputed)
            %updateCollisionState Update the displayed collision state for the currently selected body

            % If collision is computed, populate the state fields. If not,
            % reset them to their not-run values.
            if isCollisionComputed
                obj.populateCollisionState(modelSceneObjectsMap);
            else
                obj.resetCollisionState();
            end
        end

        function resetCollisionState(obj)
            %resetCollisionState Reset the collision state display

            obj.resetViewCollisionState();
        end
    end

    % Methods built into the base class
    methods
        function set.Visible(obj, isVisible)
            %set.Visible Set the view to be visible or hidden

            obj.Grid.Visible = isVisible;
        end

        function sendNotification(obj, evtName, evtData)
            %sendNotification Route a notification through a different object

            obj.SendNotificationCB(evtName, evtData);
        end
    end

    methods (Abstract)
        createInspectorPanels(obj)
        %createInspectorPanels Create data to populate the inspector view

        setup(obj)
        %setup Set up data members for next use. This is called at the end of the constructor

        populateSelectionProperties(obj, objKey, sceneObjMap, sceneModel)
        %populateSelectionProperties Populate view with properties and states for a selected object

        updateViewGivenNewRBTConfig(obj, evt)
        %updateViewGivenNewRBTConfig Update the view given a new joint configuration

        populateCollisionState(obj, modelSceneObjectsMap)
        %populateCollisionState Populate the displayed collision state fields for the currently selected object

        resetViewCollisionState(obj)
        %resetViewCollisionState Reset the collision state display
    end

    % Shared static methods
    methods (Static, Access = protected)
        function vectorText = printVector(vectorValue)
            %printVector Convert vector to a string that stays meaningful as any element gets small
            %   To ensure that the output is compact, keep all notation to
            %   two digits. However, to ensure that this doesn't result in
            %   meaningless zero-vectors when the data are small, use
            %   scientific notation if values are between 0 and 0.1 (i.e.
            %   the order of magnitude in which 2 decimals would start to
            %   be problematic for understanding).

            boundForSciNotationInOutput = 0.1;
            if any(abs(vectorValue) < boundForSciNotationInOutput & abs(vectorValue) > 0)
                vectorContentsText = char(sprintf("%0.2e, ", vectorValue));
            else
                vectorContentsText = char(sprintf("%0.2f, ", vectorValue));
            end

            % Remove trailing data and append brackets
            vectorText = "(" + string(vectorContentsText(1:end-2)) + ")";
        end

        function massText = printNumber(numValue)
            %printNumber Convert number to a string that stays meaningful as number gets small
            %   To ensure that the output is compact, keep all notation to
            %   two digits. However, to ensure that this doesn't result in
            %   meaningless zero-vectors when the data are small, use
            %   scientific notation if values are between 0 and 0.1 (i.e.
            %   the order of magnitude in which 2 decimals would start to
            %   be problematic for understanding).

            boundForSciNotationInOutput = 0.1;
            if abs(numValue) < boundForSciNotationInOutput && numValue ~= 0
                massText = sprintf("%0.2e", numValue);
            else
                massText = sprintf("%0.2f", numValue);
            end
        end
    end

    methods (Static, Access = private)
        function grid = createBaseGrid(parentFig)
            %createBaseGrid Create the grid that the rest of the inspector view use
            %   This method returns a variable so that the grid can be
            %   declared as immutable (not the case of the grid is set in
            %   this method).

            grid = uigridlayout(parentFig, 'ColumnWidth', {'1x'}, 'RowHeight', ["fit","fit"]);
            grid.Scrollable = 'on';

        end
    end
end

