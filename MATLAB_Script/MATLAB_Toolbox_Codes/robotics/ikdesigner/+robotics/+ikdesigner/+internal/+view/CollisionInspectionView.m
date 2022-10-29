classdef CollisionInspectionView < handle
%This class is for internal use only. It may be removed in the future.

%   CollisionInspectionView Object to add collision inspection to a panel view

%   Copyright 2021 The MathWorks, Inc.

    properties

        InspectedObjectKey
    end

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        SendNotificationCB

    end

    % Inspector gadgets
    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.InspectorCollisionViewTester})

        %CollisionStatusTextLabel Text indicating body's collision status
        CollisionStatusTextLabel

        %CollisionsStateList List of collision bodies
        CollisionsStateList

        %InspectCollidingBodyButton Button to go to a different body selection
        InspectCollidingBodyButton
    end

    properties (Constant)
        % Collision-has-not-been-checked text and tooltip
        COLLISIONUNCHECKEDTEXT = string(message('robotics:ikdesigner:sceneinspector:CollisionsUncheckedText'))
        COLLISIONUNCHECKEDTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:CollisionsUncheckedTooltip'))

        % Body-is-in-collision-free text and tooltip
        INCOLLISIONTEXT = string(message('robotics:ikdesigner:sceneinspector:InCollisionText'))
        INCOLLISIONTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:InCollisionTooltip'))

        % Body-is-collision-free text and tooltip
        COLLISIONFREETEXT = string(message('robotics:ikdesigner:sceneinspector:CollisionFreeText'))
        COLLISIONFREETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:CollisionFreeTooltip'))

        % Collision status label and tooltip
        BODYSTATUSLABEL = string(message('robotics:ikdesigner:sceneinspector:BodyCollisionStatusLabel'))
        BODYSTATUSTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BodyCollisionStatusTooltip'))

        % Known collisions list label and tooltip
        KNOWNCOLLISIONSLABEL = string(message('robotics:ikdesigner:sceneinspector:KnownCollisionsLabel'))
        KNOWNCOLLISIONSTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:KnownCollisionsTooltip'))

        % Inspect body Label and tooltip
        INSPECTBODYBUTTONLABEL = string(message('robotics:ikdesigner:sceneinspector:InspectBodyLabel'))
        INSPECTBODYBUTTONTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:InspectBodyTooltip'))
    end

    methods
        function obj = CollisionInspectionView(parentGrid, notificationCallback)
        %RigidBodyInspectorView Constructor

        % Add child panels to the view
            obj.addCollisionStateUIComponents(parentGrid);
            obj.SendNotificationCB = notificationCallback;

            % Start with nothing inspected
            obj.InspectedObjectKey = string.empty;
        end

        function setup(obj)
        %setup

            obj.InspectedObjectKey = string.empty;
            obj.resetCollisionState;
        end

        function populateCollisionState(obj, modelSceneObjectsMap)
        %populateCollisionState Populate the fields that display collision state
        %   This method updates the collision indicator as well as the
        %   list of collision objects.

        % Add the list of collision items
            collidingBodyKeys = modelSceneObjectsMap(obj.InspectedObjectKey).State;
            collidingBodyNames = cellfun(@(x)(modelSceneObjectsMap(x).Name), collidingBodyKeys, 'UniformOutput', false);
            obj.CollisionsStateList.Items = collidingBodyNames;
            obj.CollisionsStateList.ItemsData = collidingBodyKeys;

            if isempty(collidingBodyKeys)
                obj.CollisionStatusTextLabel.Text = obj.COLLISIONFREETEXT;
                obj.CollisionStatusTextLabel.Tooltip = obj.COLLISIONFREETOOLTIP;
                obj.InspectCollidingBodyButton.Enable = 'off';
            else
                obj.CollisionStatusTextLabel.Text = obj.INCOLLISIONTEXT;
                obj.CollisionStatusTextLabel.Tooltip = obj.INCOLLISIONTOOLTIP;

                % Select the first item and enable the inspect-body button
                obj.CollisionsStateList.Value = obj.CollisionsStateList.ItemsData{1};
                obj.InspectCollidingBodyButton.Enable = 'on';
            end

        end

        function resetCollisionState(obj)
        %resetCollisionState Reset the collision state display
        %   Set the text to un-evaluated and clear the items list.

            obj.CollisionStatusTextLabel.Text = obj.COLLISIONUNCHECKEDTEXT;
            obj.CollisionStatusTextLabel.Tooltip = obj.COLLISIONUNCHECKEDTOOLTIP;
            obj.CollisionsStateList.Items = {};
            obj.InspectCollidingBodyButton.Enable = 'off';
        end
    end

    methods
        function inspectBody(obj)
        %inspectBody Request to select a body
        %   This method is the callback for the "Inspect body" button
        %   associated with the collision bodies list. If the user
        %   wishes to inspect one of the bodies, i.e. bring it into
        %   focus in the scene inspector (and therefore also anywhere
        %   else that syncs with selection), they click that button,
        %   which triggers this method.

            selectionEvent = robotics.ikdesigner.internal.event.ViewSelectionEventData(obj.CollisionsStateList.Value, []);
            obj.SendNotificationCB('RequestSelectBody', selectionEvent);

        end
    end

    methods (Access = private)
        function addCollisionStateUIComponents(obj, parentGrid)
        %addCollisionStateUIComponents Add the states panel

        % Use a local grid to lay out the fields
            collisionStateGrid = uigridlayout(parentGrid);
            collisionStateGrid.ColumnWidth = {'fit','fit'};
            collisionStateGrid.RowHeight = {'fit','fit','fit','fit'};

            % Add the body collision status label & indicator
            collisionStatusLabel = uilabel(collisionStateGrid);
            collisionStatusLabel.Text = obj.BODYSTATUSLABEL;
            collisionStatusLabel.Tooltip = obj.BODYSTATUSTOOLTIP;
            collisionStatusLabel.Layout.Row = 1;
            collisionStatusLabel.Layout.Column = 1;
            obj.CollisionStatusTextLabel = uilabel(collisionStateGrid);
            obj.CollisionStatusTextLabel.Layout.Row = 2;
            obj.CollisionStatusTextLabel.Layout.Column = [1 2];
            obj.CollisionStatusTextLabel.Text = obj.COLLISIONUNCHECKEDTEXT;
            obj.CollisionStatusTextLabel.Tooltip = obj.COLLISIONUNCHECKEDTOOLTIP;

            % Add the body collision list label
            collisionStatusLabel = uilabel(collisionStateGrid);
            collisionStatusLabel.Text = obj.KNOWNCOLLISIONSLABEL;
            collisionStatusLabel.Tooltip = obj.KNOWNCOLLISIONSTOOLTIP;
            collisionStatusLabel.Layout.Row = 3;
            collisionStatusLabel.Layout.Column = [1 2];

            % Add the table of collisions
            obj.CollisionsStateList = uilistbox(collisionStateGrid);
            obj.CollisionsStateList.Layout.Row = 4;
            obj.CollisionsStateList.Layout.Column = [1 2];
            obj.CollisionsStateList.Multiselect = 'off';

            % Add the button that allows users to navigate to that body
            obj.InspectCollidingBodyButton = uibutton(collisionStateGrid);
            obj.InspectCollidingBodyButton.Layout.Row = 5;
            obj.InspectCollidingBodyButton.Layout.Column = [1 2];
            obj.InspectCollidingBodyButton.Text = obj.INSPECTBODYBUTTONLABEL;
            obj.InspectCollidingBodyButton.Tooltip = obj.INSPECTBODYBUTTONTOOLTIP;
            obj.InspectCollidingBodyButton.Enable = 'off';
            obj.InspectCollidingBodyButton.ButtonPushedFcn = @(src,evt)obj.inspectBody();

        end

    end
end
