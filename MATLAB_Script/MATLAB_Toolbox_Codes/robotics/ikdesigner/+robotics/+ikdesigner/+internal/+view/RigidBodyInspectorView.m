classdef RigidBodyInspectorView < robotics.ikdesigner.internal.view.InspectorViewTemplate ...
        & robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%   RigidBodyInspectorView Inspector for rigid body objects

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = private, GetAccess = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.RigidBodyInspectorPanelTester})

        %BodyIndex Index of the body currently under inspection
        BodyIndex

        %CollisionStateView Object that maintains collision state components
        CollisionStateView

        %BodyPoseView Object that maintains body pose state visual components
        BodyPoseView

    end

    % Inspector gadgets
    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.RigidBodyInspectorPanelTester})

        %BodyNameText Body name
        BodyNameText

        %JointNameText Joint name
        JointNameText

        %MassValueText Mass value
        MassValueText

        %CoMValueText Center of Mass value
        CoMValueText

        %ParentNameText Parent body name
        ParentNameText

        %ChildrenNamesText Child body names
        ChildrenNamesText

        %InertiaValuesTable Inertia values table
        InertiaValuesTable
    end

    properties (Constant)
        %Body name label and tooltip in the inspector view
        BODYNAMELABEL = string(message('robotics:ikdesigner:sceneinspector:BodyNameLabel'))
        BODYNAMETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BodyNameTooltip'))

        %Mass label and tooltip in the inspector view
        MASSLABEL = string(message('robotics:ikdesigner:sceneinspector:MassLabel'))
        MASSTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:MassTooltip'))

        %Center of Mass in the inspector view
        COMLABEL = string(message('robotics:ikdesigner:sceneinspector:CoMLabel'))
        COMTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:CoMTooltip'))

        %Parent body label and tooltip in the inspector view
        PARENTLABEL = string(message('robotics:ikdesigner:sceneinspector:ParentLabel'))
        PARENTTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BodyParentTooltip'))

        %Child bodies label and tooltip in the inspector view
        CHILDRENLABEL = string(message('robotics:ikdesigner:sceneinspector:ChildrenLabel'))
        CHILDRENTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:ChildrenTooltip'))

        %Inertia label and tooltip in the inspector view
        INERTIALABEL = string(message('robotics:ikdesigner:sceneinspector:InertiaLabel'))
        INERTIATOOLTIP = string(message('robotics:ikdesigner:sceneinspector:InertiaTooltip'))

        %Joint name label and tooltip in the inspector view
        JOINTNAMELABEL = string(message('robotics:ikdesigner:sceneinspector:JointNameLabel'))
        JOINTNAMETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointNameTooltip'))

        %PROPSLABEL Properties panel header in the inspector view
        PROPSLABEL = string(message('robotics:ikdesigner:sceneinspector:PropertiesLabel'))

        %STATESLABEL States panel header in the inspector view
        STATESLABEL = string(message('robotics:ikdesigner:sceneinspector:StatesLabel'))
    end

    % Template methods
    methods
        function createInspectorPanels(obj)
        %createInspectorPanels Create data to populate the inspector view

        % Add panels to display joint properties and state
            obj.addPropertiesPanel();
            obj.addStatesPanel();

        end

        function setup(obj)
        %setup Set up data members for next use

        % Start with nothing inspected
            obj.InspectedObjectKey = string.empty;
            obj.BodyIndex = [];

            % Reset child views
            obj.CollisionStateView.setup();
            obj.BodyPoseView.setup();
        end

        function populateSelectionProperties(obj, objKey, sceneObjMap, sceneModel)
        %populateSelectionProperties Populate view with properties and states for a selected object
        %   When the body is first selected, this method populates the
        %   inspector views with properties and states.

        % Get additional state information from the scene model:
        % transform tree and collision state. The overall collision
        % state has more fidelity than we need; all we care about here
        % is whether or not collision has been run.
            tfTree = sceneModel.TransformTree;
            isCollisionComputed = (sceneModel.CollisionState ~= robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated);

            % Get the body name and handle from the map
            body = sceneObjMap(objKey).Handle;

            % Store key properties so that state views can be separately updated later
            obj.InspectedObjectKey = objKey;
            obj.CollisionStateView.InspectedObjectKey = objKey;
            obj.BodyIndex = body.BodyInternal.Index;

            % Populate the different elements of the view
            obj.populateBodyProperties(body);
            obj.populateBodyPoseState(tfTree);
            obj.updateCollisionState(sceneObjMap, isCollisionComputed);
        end

        function updateViewGivenNewRBTConfig(obj, evt)
        %updateViewGivenNewRBTConfig Update the view given a new joint configuration

        % Update body pose from the robot's transform tree
            obj.populateBodyPoseState(evt.TransformTree);
        end

        function populateCollisionState(obj, modelSceneObjectsMap)
        %populateCollisionState Populate the displayed collision state fields for the currently selected object

        % This is handled by the collision state view object
            obj.CollisionStateView.populateCollisionState(modelSceneObjectsMap);
        end

        function resetViewCollisionState(obj)
        %resetViewCollisionState Reset the collision state display

        % This is handled by the collision state view object
            obj.CollisionStateView.resetCollisionState();
        end
    end

    methods (Access = private)
        function populateBodyProperties(obj, body)
        %populateBodyProperties Populate the inspector with body property values

            obj.BodyNameText.Text = body.Name;

            % Some properties differ for the base
            if body.BodyInternal.ParentIndex >= 0
                obj.ParentNameText.Text = body.Parent.Name;
                obj.JointNameText.Text = body.Joint.Name;
                obj.MassValueText.Text = obj.printNumber(body.Mass);
                obj.CoMValueText.Text = obj.printVector(body.CenterOfMass);
                obj.InertiaValuesTable.Data = ...
                    [body.Inertia(1) body.Inertia(6) body.Inertia(5);...
                     body.Inertia(6) body.Inertia(2) body.Inertia(4); ...
                     body.Inertia(5) body.Inertia(4) body.Inertia(3)];
            else
                % This body is the base
                obj.ParentNameText.Text = "";
                obj.JointNameText.Text = "";
                obj.MassValueText.Text = "";
                obj.CoMValueText.Text = "";
                obj.InertiaValuesTable.Data = zeros(3,3);
            end

            % Add the child bodies
            obj.ChildrenNamesText.Text = obj.printChildrenNames(body.Children);
        end

        function populateBodyPoseState(obj, tfTree)
        %populateBodyPoseState Populate the body pose edit fields from the transform tree

            if obj.BodyIndex == 0
                bodyPose = eye(4);
            else
                bodyPose = tfTree{obj.BodyIndex};
            end

            % Update the view
            obj.BodyPoseView.populatePoseView(bodyPose);
        end
    end

    methods (Access = private)
        function addPropertiesPanel(obj)
        %addPropertiesPanel Add the properties panel

        % Add the properties panel structure
            propPanel = uipanel(obj.Grid, 'Title', obj.PROPSLABEL);
            propGrid = uigridlayout(propPanel);
            propGrid.ColumnWidth = {'1x'};
            propGrid.RowHeight = {'fit'};

            listedPropsGrid = uigridlayout(propGrid);
            listedPropsGrid.ColumnWidth = {'1x','1x'};
            listedPropsGrid.RowHeight = repmat("fit",1,7);

            % Add the body name
            bodyNameLabel = uilabel(listedPropsGrid);
            bodyNameLabel.Text = obj.BODYNAMELABEL;
            bodyNameLabel.Tooltip = obj.BODYNAMETOOLTIP;
            bodyNameLabel.Layout.Row = 1;
            bodyNameLabel.Layout.Column = 1;
            obj.BodyNameText = uilabel(listedPropsGrid);
            obj.BodyNameText.Layout.Row = 1;
            obj.BodyNameText.Layout.Column = 2;

            % Add the joint name
            jointNameLabel = uilabel(listedPropsGrid);
            jointNameLabel.Text = obj.JOINTNAMELABEL;
            jointNameLabel.Tooltip = obj.JOINTNAMETOOLTIP;
            jointNameLabel.Layout.Row = 2;
            jointNameLabel.Layout.Column = 1;
            obj.JointNameText = uilabel(listedPropsGrid);
            obj.JointNameText.Layout.Row = 2;
            obj.JointNameText.Layout.Column = 2;

            % Add the mass
            massLabel = uilabel(listedPropsGrid);
            massLabel.Text = obj.MASSLABEL;
            massLabel.Tooltip = obj.MASSTOOLTIP;
            massLabel.Layout.Row = 3;
            massLabel.Layout.Column = 1;
            obj.MassValueText = uilabel(listedPropsGrid);
            obj.MassValueText.Layout.Row = 3;
            obj.MassValueText.Layout.Column = 2;

            % Add the Center of Mass
            comLabel = uilabel(listedPropsGrid);
            comLabel.Text = obj.COMLABEL;
            comLabel.Tooltip = obj.COMTOOLTIP;
            comLabel.Layout.Row = 4;
            comLabel.Layout.Column = 1;
            obj.CoMValueText = uilabel(listedPropsGrid);
            obj.CoMValueText.Layout.Row = 4;
            obj.CoMValueText.Layout.Column = 2;

            % Add the Parent name
            parentNameLabel = uilabel(listedPropsGrid);
            parentNameLabel.Text = obj.PARENTLABEL;
            parentNameLabel.Tooltip = obj.PARENTTOOLTIP;
            parentNameLabel.Layout.Row = 5;
            parentNameLabel.Layout.Column = 1;
            obj.ParentNameText = uilabel(listedPropsGrid);
            obj.ParentNameText.Layout.Row = 5;
            obj.ParentNameText.Layout.Column = 2;

            % Add the Children names
            childBodyNamesLabel = uilabel(listedPropsGrid);
            childBodyNamesLabel.Text = obj.CHILDRENLABEL;
            childBodyNamesLabel.Tooltip = obj.CHILDRENTOOLTIP;
            childBodyNamesLabel.Layout.Row = 6;
            childBodyNamesLabel.Layout.Column = 1;
            obj.ChildrenNamesText = uilabel(listedPropsGrid);
            obj.ChildrenNamesText.Layout.Row = 6;
            obj.ChildrenNamesText.Layout.Column = 2;

            % Add the inertia grid, label, and table
            inertiaPropGrid = uigridlayout(propGrid);
            inertiaPropGrid.ColumnWidth = {150};
            inertiaPropGrid.RowHeight = {"fit",70};

            intertiaLabel = uilabel(inertiaPropGrid);
            intertiaLabel.Text = obj.INERTIALABEL;
            intertiaLabel.Tooltip = obj.INERTIATOOLTIP;
            intertiaLabel.Layout.Row = 1;
            intertiaLabel.Layout.Column = 1;

            obj.InertiaValuesTable = uitable(inertiaPropGrid, 'Data', eye(3));
            obj.InertiaValuesTable.Layout.Column = [1 2];
            obj.InertiaValuesTable.Layout.Row = 2;
            obj.InertiaValuesTable.ColumnEditable=[false, false, false];
            obj.InertiaValuesTable.ColumnWidth = {'1x','1x','1x'};
            obj.InertiaValuesTable.ColumnFormat={'numeric','numeric','numeric'};
            obj.InertiaValuesTable.RowName = [];
            obj.InertiaValuesTable.ColumnName = [];
        end

        function statesGrid = addStatesPanel(obj)
        %addStatesPanel Add panel containing pose and collision state data

            statesPanel = uipanel(obj.Grid, 'Title', obj.STATESLABEL);

            % There is one large grid containing all the states
            statesGrid = uigridlayout(statesPanel);
            statesGrid.ColumnWidth = {'1x'};
            statesGrid.RowHeight = repmat("fit",1,5);

            % Collision fields are on a separate object
            obj.BodyPoseView = robotics.ikdesigner.internal.view.PoseInspectionView(statesGrid, obj.SendNotificationCB);
            obj.CollisionStateView = robotics.ikdesigner.internal.view.CollisionInspectionView(statesGrid, obj.SendNotificationCB);
        end

    end

    methods (Static, Access = private)
        function childrenString = printChildrenNames(childBodies)
        %printChildrenNames Print a string containing the list of child body names

            childrenString = "";
            for i = 1:numel(childBodies)
                if i == numel(childBodies)
                    childrenString = childrenString + childBodies{i}.Name;
                else
                    childrenString = childrenString + childBodies{i}.Name + ", ";
                end
            end
        end
    end
end
