classdef NoSelectionInspectorView < robotics.ikdesigner.internal.view.InspectorViewTemplate
    %This class is for internal use only. It may be removed in the future.

    %   NoSelectionInspectorView Inspector view when nothing is selected

    %   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        INSTRUCTIONLABEL = string(message('robotics:ikdesigner:sceneinspector:NoSelectionLabel').getString);
    end

    methods
        function createInspectorPanels(obj)
            %createInspectorPanels Create data to populate the inspector view

            %   Add text to the view so that when the view is empty, this
            %   text can help instruct users on why that is the case.
            instructionLabel = uilabel(obj.Grid);
            instructionLabel.WordWrap = 'on';
            instructionLabel.Text = obj.INSTRUCTIONLABEL;
            instructionLabel.Visible = 'on';
            instructionLabel.Enable = 'on';
        end

        function setup(~)
            %setup Set up parameters
            %   This is a no-op for this view since no objects are selected

        end

        function populateSelectionProperties(~, ~, ~, ~)
            %populateSelectionProperties Populate view with properties and states for a selected object
            %   This is a no-op for this view since no objects are selected
        end

        function updateViewGivenNewRBTConfig(~, ~)
            %updateViewGivenNewRBTConfig Update the view given a new joint configuration
            %   This is a no-op for this view since no objects are selected
        end

        function populateCollisionState(~, ~)
            %populateCollisionState Populate the displayed collision state fields for the currently selected object
            %   This is a no-op for this view since no objects are selected
        end

        function resetViewCollisionState(~)
            %resetViewCollisionState Reset the collision state display
            %   This is a no-op for this view since no objects are selected
        end
    end
end

