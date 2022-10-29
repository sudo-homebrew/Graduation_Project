classdef SceneInspectorView < robotics.ikdesigner.internal.view.View
    %This function is for internal use only. It may be removed in the future.
    
    %   SceneInspectorView View that details the attributes of selected items in the scene
    
    %   Copyright 2021 The MathWorks, Inc.

    events
        %RequestSelectBody Event to select a body in the scene
        RequestSelectBody

        %RequestMoveJoint Event to move a joint in the configuration
        RequestMoveJoint
    end
    
    properties (SetAccess = private, GetAccess = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.SceneInspectorUITester})
        
        %Parent Handle to parent UIFigure
        Parent

        %NoSelectionView View when no object is selected
        NoSelectionView
        
        %RigidBodyView Inspector view for rigid body object
        RigidBodyView
        
        %RevoluteJointView Inspector view for rigid body revolute joints
        RevoluteJointView
        
        %PrismaticJointView Inspector view for rigid body prismatic joints
        PrismaticJointView
        
        %FixedJointView Inspector view for rigid body fixed joints
        FixedJointView
        
        %CollisionView Inspector view for collision object
        CollisionView

        %ActiveView
        ActiveView
    end

    % Constant character arrays used to identify joint types
    properties (Constant)
        REVJTTYPETEXT = 'revolute'

        PRISJTTYPETEXT = 'prismatic'

        FIXEDJTTYPETEXT = 'fixed'
    end
    
    methods
        function obj = SceneInspectorView(fig)
            %SceneInspectorView Constructor
            
            % Create a grid and initialize a browser view
            obj.Parent = fig;

            % Create the view options
            notificationCB = @(evt, evtdata)notify(obj, evt, evtdata);
            obj.RigidBodyView = robotics.ikdesigner.internal.view.RigidBodyInspectorView(obj, fig, notificationCB);
            obj.RevoluteJointView = robotics.ikdesigner.internal.view.RevoluteJointInspectorView(obj, fig, notificationCB);
            obj.PrismaticJointView = robotics.ikdesigner.internal.view.PrismaticJointInspectorView(obj, fig, notificationCB);
            obj.FixedJointView = robotics.ikdesigner.internal.view.FixedJointInspectorView(obj, fig, notificationCB);
            obj.CollisionView = robotics.ikdesigner.internal.view.CollisionObjectInspectorView(obj, fig, notificationCB);
            obj.NoSelectionView = robotics.ikdesigner.internal.view.NoSelectionInspectorView(obj, fig, notificationCB);

            % Active the no-selection view
            obj.updateActiveView(obj.NoSelectionView);
        end
        
        function setup(obj)
            %setup Set up view for new session

            obj.updateActiveView(obj.NoSelectionView);
        end
        
        function initialize(~)
            %initialize
        end

        function updateSelection(obj, event)
            %updateSelection Update the scene inspector from a new selection event

            % Get model from the event source
            sceneModel = event.Source;

            % Get the core objects needed to populate the view:
            % the key and map
            selectionKey = event.SceneObjectKey;
            modelSceneObjMap = sceneModel.SceneObjectsMap;

            % Route the event to the proper inspector view
            selectionHandle = modelSceneObjMap(selectionKey).Handle;
            if isa(selectionHandle, 'robotics.core.internal.CollisionGeometryBase')
                activeView = obj.CollisionView();
            elseif isa(selectionHandle, 'rigidBody')
                activeView = obj.RigidBodyView();
            elseif isa(selectionHandle, 'rigidBodyJoint') && strcmp(selectionHandle.Type, obj.REVJTTYPETEXT)
                activeView = obj.RevoluteJointView();
            elseif isa(selectionHandle, 'rigidBodyJoint') && strcmp(selectionHandle.Type, obj.PRISJTTYPETEXT)
                activeView = obj.PrismaticJointView();
            elseif isa(selectionHandle, 'rigidBodyJoint') && strcmp(selectionHandle.Type, obj.FIXEDJTTYPETEXT)
                activeView = obj.FixedJointView();
            else
                activeView = obj.NoSelectionView();
            end

            % Populate the view and make it visible
            activeView.populate(selectionKey, modelSceneObjMap, event.Source);
            obj.updateActiveView(activeView);
        end

        function updateRBTConfiguration(obj, evt)
            %updateRBTConfiguration Update view when robot's joint configuration state changes

            obj.ActiveView.updateRBTConfiguration(evt);

        end

        function updateSceneContent(obj, changedSceneContentData)

            % Check if the currently selected object still exists in the
            % scene. If not, go to the no-selection view.
            doesSelectionExist = isempty(obj.ActiveView.InspectedObjectKey) || changedSceneContentData.SceneObjectsMap.isKey(obj.ActiveView.InspectedObjectKey);
            if ~doesSelectionExist
                activeView = obj.NoSelectionView();
                obj.updateActiveView(activeView);
            end

            % Reset collision state
            obj.resetCollisionState();

        end

        function updateCollisionState(obj, evt)
            %updateCollisionState Update view when the scene collision state changes

            isCollisionComputed = (evt.RBTCollisionState ~= robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated);
            obj.ActiveView.updateCollisionState(evt.SceneObjectsMap, isCollisionComputed);

        end

        function resetCollisionState(obj)
            %resetCollisionState Update view when the scene collision state is reset

            obj.ActiveView.resetCollisionState();

        end

        function updateActiveView(obj, view)
            %set.ActiveView Define the active view
            %   Turn all views to invisible except for the active one

            obj.RigidBodyView.Visible = false;
            obj.RevoluteJointView.Visible = false;
            obj.PrismaticJointView.Visible = false;
            obj.FixedJointView.Visible = false;
            obj.CollisionView.Visible = false;
            obj.NoSelectionView.Visible = false;

            view.Visible = true;
            obj.ActiveView = view;
        end
    end
end

