classdef SceneController < robotics.manip.internal.InternalAccess
    %This class if for internal use only and may be removed in a future release
    
    %SceneController Controller that listens to SceneModel
    %   The SceneController has listeners on the scene model and views
    %   that pertain to the scene. It facilitates communication
    %   regarding the contents and state of the scene.
    
    %   Copyright 2021-2022 The MathWorks, Inc.
    
    properties (Access = ?matlab.unittest.TestCase)
        
        %SceneModel Model containing scene contents and state data
        SceneModel
        
        %ConfigModel Model containing stored configuration data
        ConfigModel
        
        %SolverModel Model containing IK state data
        SolverModel
        
        %SceneCanvasView Handle to the scene canvas view
        SceneCanvasView
        
        %SceneInspectorView Handle to the scene inspector view
        SceneInspectorView
        
        %SceneBrowserView Handle to the scene browser view
        SceneBrowserView
        
        %StatusBarView Handle to the status bar view
        StatusBarView

        %ToolstripView Handle to toolstrip view
        ToolstripView

        %ConstraintsBrowserView Handle to toolstrip view
        ConstraintsBrowserView

        %AppWindowView Handle to the app window
        AppWindowView
    end
    
    methods
        function obj = SceneController(models, views)
            %SceneController Constructor
            
            % The scene controller only has model event listeners on the
            % scene model
            obj.SceneModel = models.SceneModel;
            
            % Other models that may receive commands
            obj.ConfigModel = models.ConfigModel;
            obj.SolverModel = models.SolverModel;
            
            % Views that have listeners
            obj.SceneCanvasView = views.SceneCanvasView;
            obj.SceneInspectorView = views.SceneInspectorView;
            obj.SceneBrowserView = views.SceneBrowserView;
            obj.StatusBarView = views.StatusBarView;
            obj.ToolstripView = views.ToolstripView;
            obj.ConstraintsBrowserView = views.ConstraintsBrowserView;
            obj.AppWindowView = views.Window;
            
        end
        
        function initialize(obj)
            obj.addViewListeners;
            obj.addModelListeners;
        end
    end
    
    methods (Access = private)
        
        function addViewListeners(obj)
            %addViewListeners Add listeners for events that occur on views

            addlistener(obj.SceneCanvasView, 'ObjectSelectionChange', @(source, event)obj.updateSelectedSceneObject(event) );
            addlistener(obj.SceneBrowserView, 'ObjectSelectionChange', @(source, event)obj.updateSelectedSceneObject(event) ); 
            addlistener(obj.SceneBrowserView, 'ObjectDeleteRequest', @(source, event)obj.deleteObjectFromModel(event) );
            addlistener(obj.SceneBrowserView, 'RequestMarkerBodyChange', @(source, event)obj.updateMarkerBodyAssignment(event) );
            addlistener(obj.SceneInspectorView, 'RequestSelectBody', @(source, event)obj.updateSelectedSceneObject(event) ); 
            addlistener(obj.SceneInspectorView, 'RequestMoveJoint', @(source, event)obj.updateJointPositionsInModel(event) ); 
            addlistener(obj.ToolstripView, 'ObjectSelectionChange', @(source, event)obj.updateSelectedSceneObject(event) );
            addlistener(obj.ToolstripView, 'ConstraintVisualsChange', @(source, event)obj.addModelDataToSceneConstraintVisual(event) );
            addlistener(obj.ToolstripView, 'MarkerBodySelected', @(source, event)obj.updateMarkerBodyAssignment(event) );
            addlistener(obj.ToolstripView, 'CheckCurrentConfigCollisions', @(source, event)obj.checkCurrentConfigCollisions() );
            addlistener(obj.ToolstripView, 'CheckAllConfigsCollisions', @(source, event)obj.checkAllConfigsCollision() );
            addlistener(obj.ToolstripView, 'IgnoreSelfCollisions', @(source, event)obj.enableIgnoreSelfCollision(true) );
            addlistener(obj.ToolstripView, 'IncludeSelfCollisions', @(source, event)obj.enableIgnoreSelfCollision(false) );
            addlistener(obj.ConstraintsBrowserView, 'RequestMarkerDisplayToggle', @(source, event)obj.toggleCanvasPoseMarker() );
            
        end
        
        function addModelListeners(obj)
            %addModelListeners Add listeners for events that occur on models

            addlistener(obj.SceneModel, 'RBTStateChanged', @(source, event)obj.updateViewsWithNewRBTConfig(event) );
            addlistener(obj.SceneModel, 'SceneContentsChanged', @(source, event)obj.updateViewsWithNewSceneContents(event) );
            addlistener(obj.SceneModel, 'SceneSelectionChanged', @(source, event)obj.updateAllSceneSelections(event) );
            addlistener(obj.SceneModel, 'ConstraintBodySelectionChanged', @(source, event)obj.updateConstraintBodySelection(event) );
            addlistener(obj.SceneModel, 'CollisionStateChanged', @(source, event)obj.updateViewsWithNewCollisionState(event) );
            addlistener(obj.SceneModel, 'ConstraintSceneDisplayUpdated', @(source, event)obj.updateSceneWithDisplayOverlay(event) );
            addlistener(obj.SceneModel, 'MarkerBodyUpdated', @(source, event)obj.updateSolverEEBody() );
            
        end

        function enableIgnoreSelfCollision(obj, isEnabled)
            %enableIgnoreSelfCollision Choose whether or not to ignore self collision in collision checks

            obj.SceneModel.enableIgnoreSelfCollision(isEnabled)
        end
        
        function checkAllConfigsCollision(obj)
            %checkAllConfigsCollision Check collision status of all stored configurations

            obj.setAppWindowBusyState(true);
            keys = obj.ConfigModel.AllConfigKeys;
            for i = 1:numel(keys)
                configKey = keys(i);
                config = obj.ConfigModel.getConfig(configKey);
                [overallCollisionState, areBodiesColliding] = obj.SceneModel.checkCollision(config);
                obj.ConfigModel.updateCollisionState(configKey, overallCollisionState, areBodiesColliding);
            end

            % Also check the current configuration, which may or may not be
            % stored
            obj.checkCurrentConfigCollisions();
            obj.setAppWindowBusyState(false);
        end
        
        function checkCurrentConfigCollisions(obj)
            %checkCurrentConfigCollisions Check collision status for current configuration

            obj.SceneModel.checkCurrentCollision();
        end
        
        function updateSelectedSceneObject(obj, event)
            %updateSelectedSceneObject Change the selected object in the scene model

            obj.SceneModel.updateSceneSelection(event);
        end
        
        function updateMarkerBodyAssignment(obj, event)
            %updateMarkerBodyAssignment Assign the marker to a new body in the scene

            obj.SceneModel.updateSceneMarkerBody(event);
        end

        function updateJointPositionsInModel(obj, evt)
            %updateJointPositionsInModel Update individual joint positions in the model

            config = obj.SceneModel.Config;
            config(evt.JointIndex) = evt.Value;
            obj.SolverModel.applyJointConfiguration(config);

            eeBodyPose = obj.SolverModel.LastSolutionEEPose;
            obj.SceneCanvasView.updateMarkerPose(eeBodyPose);

        end
        
        function updateSolverEEBody(obj)
            %updateSolverEEBody Update the body associated with the main pose target constraint on the solver

            markerBodyName = obj.SceneModel.MarkerBodyName;
            obj.SolverModel.updateEEPoseBody(markerBodyName);
        end
        
        function updateAllSceneSelections(obj, selectionEvent)
            %updateAllSceneSelections Update selected item in all scene views
            %   This method triggers a selection update in all the scene
            %   components so that the selected body or other item is synced
            %   between the different scene views.

            obj.SceneCanvasView.updateSelection(selectionEvent);
            obj.SceneBrowserView.updateSelection(selectionEvent);
            obj.SceneInspectorView.updateSelection(selectionEvent);
        end

        function updateConstraintBodySelection(obj, selectionEvent)

            obj.SceneCanvasView.updateSelection(selectionEvent);
        end
        
        function updateViewsWithNewRBTConfig(obj, eventData)
            %updateViewsWithNewRBTConfig Update views with new joint configuration
            %   When the state of the joint configuration changes, this
            %   model directs that action to the appropriate views. This
            %   method just updates joint configuration; any downstream
            %   actions like resetting collisions are handled via separate
            %   notifications sent from the calling action.

            obj.SceneCanvasView.updateRBTConfiguration(eventData);
            obj.SceneInspectorView.updateRBTConfiguration(eventData);
            obj.StatusBarView.update(obj.ConfigModel.ConfigName, obj.SceneModel.Config);
        end

        function updateViewsWithNewSceneContents(obj, changedSceneContentData)
            %updateViewsWithNewSceneContents Reflect added scene object in scene views 

            obj.SceneCanvasView.updateSceneContent(changedSceneContentData);
            obj.SceneBrowserView.updateSceneContent(changedSceneContentData);
            obj.SceneInspectorView.updateSceneContent(changedSceneContentData);
            obj.SceneInspectorView.resetCollisionState();

            % Update the configuration model so that the configurations
            % view is updated if applicable
            obj.ConfigModel.resetAllCollisionStates();
        end

        function deleteObjectFromModel(obj, deleteEvent)
            %deleteObjectFromModel

            obj.SceneModel.removeObjectFromSceneModel(deleteEvent);
        end
        
        function updateViewsWithNewCollisionState(obj, changedCollisionStateEvent)
            %updateViewsWithNewCollisionState Change the collision state displayed in the views
            
            obj.SceneCanvasView.updateCollisionState(changedCollisionStateEvent);
            obj.SceneBrowserView.updateCollisionState(changedCollisionStateEvent);
            obj.SceneInspectorView.updateCollisionState(changedCollisionStateEvent);

            % Update the configuration model so that the configurations
            % view is updated if applicable
            currCollState = obj.SceneModel.CollisionState;
            currCollRawData = obj.SceneModel.CollisionRawData;
            obj.ConfigModel.updateCurrentConfigCollision(currCollState, currCollRawData);
        end

        function addModelDataToSceneConstraintVisual(obj, eventData)
            %addModelDataToSceneConstraintVisual Add model data to scene constraint

            obj.SceneModel.addModelDataToSceneConstraintVisual(eventData);
        end

        function updateSceneWithDisplayOverlay(obj, eventData)
            %updateSceneWithDisplayOverlay Update the scene canvas with constraint overlay visuals

            obj.SceneCanvasView.updateSceneWithAxesOverlay(eventData);
        end

        function toggleCanvasPoseMarker(obj)
            %toggleCanvasPoseMarker Toggle marker pose visibility
            
            obj.SceneCanvasView.toggleMarkerView();
        end

        function setAppWindowBusyState(obj, isBusy)
            %setAppWindowBusyState Set the app window to be busy or clear
            
            obj.AppWindowView.Busy = isBusy;
        end

    end
end
