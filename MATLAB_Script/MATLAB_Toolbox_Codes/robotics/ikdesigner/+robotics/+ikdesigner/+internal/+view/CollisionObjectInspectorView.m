classdef CollisionObjectInspectorView < robotics.ikdesigner.internal.view.InspectorViewTemplate
%This class is for internal use only. It may be removed in the future.

%   CollisionObjectInspectorView Inspector view for collision objects

%   Copyright 2021 The MathWorks, Inc.

    properties
        %NameString
        NameString

        %TypeString
        TypeString

        %CollisionPropNames Cell array of collision dimension property names
        CollisionPropNames = {}

        %CollisionPropValues Cell array of collision dimension property values
        CollisionPropValues

        %CollisionStateView Object that displays the collision state
        CollisionStateView

        %BodyPoseView Object that displays pose
        BodyPoseView
    end

    % Labels, provided as strings. In some cases these labels are
    % concatenated in arrays that assume a string format
    properties (Constant)

        % Body name label and tooltip
        NAMELABEL = string(message('robotics:ikdesigner:sceneinspector:NameLabel'))
        NAMETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:NameTooltip'))

        % Collision body type label and tooltip
        TYPELABEL = string(message('robotics:ikdesigner:sceneinspector:TypeLabel'))
        TYPETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:TypeTooltip'))

        % Vertices label and tooltip
        VERTICESLABEL = string(message('robotics:ikdesigner:sceneinspector:VerticesLabel'))
        VERTICESTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:VerticesTooltip'))

        % Box x dimension label and tooltip
        BOXXLABEL = string(message('robotics:ikdesigner:sceneinspector:BoxXLabel'))
        BOXXTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BoxXTooltip'))

        % Box y dimension label and tooltip
        BOXYLABEL = string(message('robotics:ikdesigner:sceneinspector:BoxYLabel'))
        BOXYTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BoxYTooltip'))

        % Box z dimension label and tooltip
        BOXZLABEL = string(message('robotics:ikdesigner:sceneinspector:BoxZLabel'))
        BOXZTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:BoxZTooltip'))

        % Radius label and tooltip
        RADIUSLABEL = string(message('robotics:ikdesigner:sceneinspector:RadiusLabel'))
        RADIUSTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:RadiusTooltip'))

        % Length label and tooltip
        LENGTHLABEL = string(message('robotics:ikdesigner:sceneinspector:LengthLabel'))
        LENGTHTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:LengthTooltip'))

        % Collision states label and tooltip
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
        %setup Set up properties

        % Start with nothing inspected
            obj.InspectedObjectKey = string.empty;

            % Reset child views
            obj.CollisionStateView.setup();
            obj.BodyPoseView.setup();
        end

        function populateSelectionProperties(obj, objKey, sceneObjMap, sceneModel)
        %populateSelectionProperties Populate view with properties and states for a selected body
        %   When the body is first selected, this method populates the
        %   inspector views with properties and states.

        % Store key properties so that state views can be separately updated later
            obj.InspectedObjectKey = objKey;
            obj.CollisionStateView.InspectedObjectKey = objKey;

            % Get additional state information from the scene model:
            % collision state. The overall collision state has more
            % fidelity than we need; all we care about here is whether or
            % not collision has been run.
            isCollisionComputed = (sceneModel.CollisionState ~= robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated);

            % Get the body name and handle from the map
            collisionBody = sceneObjMap(objKey).Handle;
            bodyName = sceneObjMap(objKey).Name;

            % Populate the different sections of the inspector
            obj.populateBodyProperties(bodyName, collisionBody);
            obj.updateCollisionState(sceneObjMap, isCollisionComputed);
        end

        function updateViewGivenNewRBTConfig(~, ~)
        %updateViewGivenNewRBTConfig Update the view given a new joint configuration
        %   This is a no-op since the robot's joint configuration has
        %   no impact on the collision object body properties or state.
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
        function populateBodyProperties(obj, bodyName, collisionBody)
        %populateBodyProperties Populate the inspector with body property values

            obj.NameString.Text = bodyName;
            obj.TypeString.Text = class(collisionBody);
            obj.populateBodyDimensions(collisionBody);
            obj.BodyPoseView.populatePoseView(collisionBody.Pose);
        end

        function populateBodyDimensions(obj, collisionBody)
        %populateBodyDimensions Populate collision body dimensions
        %   The different collision bodies have different ways of
        %   specifying dimensions, which must be handled when they are
        %   populated.

            switch class(collisionBody)
              case 'collisionMesh'
                propNames = obj.VERTICESLABEL;
                propTooltips = obj.VERTICESTOOLTIP;
                propVals = obj.describeContents(collisionBody.Vertices);
              case 'collisionBox'
                propNames = [obj.BOXXLABEL obj.BOXYLABEL obj.BOXZLABEL];
                propTooltips = [obj.BOXXTOOLTIP obj.BOXYTOOLTIP obj.BOXZTOOLTIP];
                propVals = [...
                    obj.printNumber(collisionBody.X) ...
                    obj.printNumber(collisionBody.Y) ...
                    obj.printNumber(collisionBody.Z) ];
              case 'collisionCylinder'
                propNames = [obj.RADIUSLABEL obj.LENGTHLABEL];
                propTooltips = [obj.RADIUSTOOLTIP obj.LENGTHTOOLTIP];
                propVals = [...
                    obj.printNumber(collisionBody.Radius) ...
                    obj.printNumber(collisionBody.Length)];
              case 'collisionSphere'
                propNames = obj.RADIUSLABEL;
                propTooltips = obj.RADIUSTOOLTIP;
                propVals = obj.printNumber(collisionBody.Radius);
            end

            % Iterate over the list of property names to both display the
            % label and value, and enable the view
            for i = 1:numel(propNames)
                obj.CollisionPropNames{i}.Text = propNames(i);
                obj.CollisionPropNames{i}.Tooltip = propTooltips(i);
                obj.CollisionPropValues{i}.Text = propVals(i);
                obj.CollisionPropNames{i}.Visible = 'on';
                obj.CollisionPropValues{i}.Visible = 'on';
            end

            % Hide the remaining unused property labels
            for j = (i+1):numel(obj.CollisionPropNames)
                obj.CollisionPropNames{j}.Visible = 'off';
                obj.CollisionPropValues{j}.Visible = 'off';
            end

        end

        function addPropertiesPanel(obj)
        %addPropertiesPanel Add the properties panel

            propPanel = uipanel(obj.Grid, 'Title', 'Properties');
            propGrid = uigridlayout(propPanel);
            propGrid.ColumnWidth = {'1x'};
            propGrid.RowHeight = {'fit','1x'};

            % Add the properties forming a vertical list
            obj.addListedProps(propGrid);

            % Add the pose properties
            poseGrid = uigridlayout(propGrid);
            poseGrid.ColumnWidth = {'1x'};
            poseGrid.RowHeight = {'fit','fit'};
            poseGrid.Layout.Row = 2;
            poseGrid.Layout.Column = 1;
            obj.BodyPoseView = robotics.ikdesigner.internal.view.PoseInspectionView(poseGrid, obj.SendNotificationCB);
        end

        function statesGrid = addStatesPanel(obj)
        %addStatesPanel Add panel containing pose and collision state data

            statesPanel = uipanel(obj.Grid, 'Title', obj.STATESLABEL);

            % There is one large grid containing all the states
            statesGrid = uigridlayout(statesPanel);
            statesGrid.ColumnWidth = {'1x'};
            statesGrid.RowHeight = repmat("fit",1,5);

            % Add collision fields view a targeted view
            obj.CollisionStateView = robotics.ikdesigner.internal.view.CollisionInspectionView(statesGrid, obj.SendNotificationCB);
        end

        function addListedProps(obj, propGrid)
        %addListedProps Add properties that form a vertical list

            import robotics.ikdesigner.internal.helpers.*

            % Set up a grid for the list
            listpropsGrid = uigridlayout(propGrid);
            listpropsGrid.Layout.Row = 1;
            listpropsGrid.Layout.Column = 1;
            listpropsGrid.ColumnWidth = {'1x','1x'};
            listpropsGrid.RowHeight = {'fit','fit','fit','fit','fit'};

            % Body name label and text
            bodyNameLabel = uilabel(listpropsGrid);
            bodyNameLabel.Text = obj.NAMELABEL;
            bodyNameLabel.Tooltip = obj.NAMETOOLTIP;
            bodyNameLabel.Layout.Row = 1;
            bodyNameLabel.Layout.Column = 1;
            obj.NameString = uilabel(listpropsGrid);
            obj.NameString.Text = '';
            obj.NameString.Layout.Row = 1;
            obj.NameString.Layout.Column = 2;

            % Body type label and text
            bodyTypeLabel = uilabel(listpropsGrid);
            bodyTypeLabel.Text = obj.TYPELABEL;
            bodyTypeLabel.Tooltip = obj.TYPETOOLTIP;
            bodyTypeLabel.Layout.Row = 2;
            bodyTypeLabel.Layout.Column = 1;
            obj.TypeString = uilabel(listpropsGrid);
            obj.TypeString.Text = '';
            obj.TypeString.Layout.Row = 2;
            obj.TypeString.Layout.Column = 2;

            % Mesh property 1
            obj.CollisionPropNames{1} = uilabel(listpropsGrid);
            obj.CollisionPropNames{1}.Text = obj.BOXXLABEL;
            obj.CollisionPropNames{1}.Tooltip = obj.BOXXTOOLTIP;
            obj.CollisionPropNames{1}.Layout.Row = 3;
            obj.CollisionPropNames{1}.Layout.Column = 1;
            obj.CollisionPropValues{1} = uilabel(listpropsGrid);
            obj.CollisionPropValues{1}.Text = '';
            obj.CollisionPropValues{1}.Layout.Row = 3;
            obj.CollisionPropValues{1}.Layout.Column = 2;

            % Mesh property 2
            obj.CollisionPropNames{2} = uilabel(listpropsGrid);
            obj.CollisionPropNames{2}.Text = obj.BOXYLABEL;
            obj.CollisionPropNames{2}.Tooltip = obj.BOXYTOOLTIP;
            obj.CollisionPropNames{2}.Layout.Row = 4;
            obj.CollisionPropNames{2}.Layout.Column = 1;
            obj.CollisionPropValues{2} = uilabel(listpropsGrid);
            obj.CollisionPropValues{2}.Text = '';
            obj.CollisionPropValues{2}.Layout.Row = 4;
            obj.CollisionPropValues{2}.Layout.Column = 2;

            % Mesh property 3
            obj.CollisionPropNames{3} = uilabel(listpropsGrid);
            obj.CollisionPropNames{3}.Text = obj.BOXZLABEL;
            obj.CollisionPropNames{3}.Tooltip = obj.BOXZTOOLTIP;
            obj.CollisionPropNames{3}.Layout.Row = 5;
            obj.CollisionPropNames{3}.Layout.Column = 1;
            obj.CollisionPropValues{3} = uilabel(listpropsGrid);
            obj.CollisionPropValues{3}.Text = '';
            obj.CollisionPropValues{3}.Layout.Row = 5;
            obj.CollisionPropValues{3}.Layout.Column = 2;
        end
    end

    methods (Static, Access = private)
        function descrString = describeContents(numVar)
        %describeContents Return the string that defines variable size and type
            varDim = size(numVar);
            varClass = class(numVar);
            descrString = sprintf("%i x %i %s", varDim(1), varDim(2), varClass);
        end
    end
end
