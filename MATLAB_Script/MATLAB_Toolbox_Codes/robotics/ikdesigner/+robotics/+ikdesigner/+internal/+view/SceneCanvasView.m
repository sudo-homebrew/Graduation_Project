classdef SceneCanvasView < robotics.ikdesigner.internal.view.View
    %This class is for internal use only. It may be removed in the future.
    
    %SceneCanvas View that displays the robot and scene objects
    
    %   Copyright 2021 The MathWorks, Inc.
    
    events
        %MarkerButtonMove Event to indicate the control marker is selected and moving
        MarkerButtonMove
        
        %MarkerButtonMove Event to indicate the control marker has been released
        MarkerButtonUp
        
        %ObjectSelectionChange Event to indicate that a new object has been clicked on
        ObjectSelectionChange
        
    end
    
    properties        
        Parent
        
        Axes
        
        FigStateHandler

        Marker

        AxesOverlayStrategy
    end

    properties (Dependent)
        %MarkerPose Pose of the marker in the canvas
        MarkerPose
    end
    
    methods
        function obj = SceneCanvasView()
            %SceneCanvas

        end
        
        function setup(obj, parentFigure, robot, sceneObjectsMap, rigidBodyKeysMap, eeBodyPose)
            %SceneCanvas
            
            obj.Parent = parentFigure;
            obj.Parent.AutoResizeChildren = 'on';
            
            % Set up the figure axes
            % TODO: Modify show so it can be used with a uigrid
            % grid = uigridlayout(obj.Parent, 'ColumnWidth', {'1x'}, 'Rowheight', {'1x'});
            obj.Axes = uiaxes(obj.Parent);
            
            % Create a figure state handler for the object
            obj.Axes.Units = 'normalized';
            obj.FigStateHandler = robotics.ikdesigner.internal.interactive.FigureStateHandler(sceneObjectsMap, rigidBodyKeysMap, robot, obj.Axes, @(objHandle)obj.objectSelectionChangeCB(objHandle));

            % Assign an IK control marker to the axes at the desired target pose
            obj.Marker = robotics.manip.internal.IKControlMarker(obj.Axes, eeBodyPose, 1, 'ExternalButtonDownCB', @(tgtPose)obj.markerButtonMoveCB(tgtPose), 'ExternalButtonUpCB', @(tgtPose)obj.markerButtonUpCB(tgtPose));
            
        end
                
        function initialize(~)
            
        end

        function updateSceneWithAxesOverlay(obj, evt)
            %updateSceneWithAxesOverlay Add an overlay to the axes in the scene canvas view

            switch evt.UpdateType
                case robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.NewOverlay
                    if ~isempty(obj.AxesOverlayStrategy)
                        obj.AxesOverlayStrategy.delete();
                    end
                    obj.AxesOverlayStrategy = evt.AxesOverlayConstructorHandle(evt, obj.FigStateHandler);
                case robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DeleteOverlay
                    obj.AxesOverlayStrategy.delete();
                    obj.AxesOverlayStrategy = [];
                otherwise
                    obj.AxesOverlayStrategy.update(evt);                    
            end

        end
        
        function objectSelectionChangeCB(obj, objKey)
            %objectSelectionChangeCB Callback for object selection in the scene
            %   This callback is triggered when the user selects a rigid
            %   body or collision object. The callback sends a message to
            %   the appropriate listener to indicate the name and type of
            %   item selected in the canvas.

            event = robotics.ikdesigner.internal.event.ViewSelectionEventData(objKey, []);
            notify(obj, "ObjectSelectionChange", event);
        end
        
        function updateRBTConfiguration(obj, configurationUpdateEvent)
            %updateRBTConfiguration Update the configuration of the displayed rigid body tree object
            %   This method directs the figure state handler to update the
            %   HG transforms for a specific configuration. The method
            %   passes the transform tree, which is closely connected to
            %   the actual transforms the HG transform objects will use to
            %   update their relative figure pose.

            % Move the robot
            obj.FigStateHandler.updateAxesConfig(configurationUpdateEvent.TransformTree);

            % Update axes overlay
            if ~isempty(obj.AxesOverlayStrategy)
                obj.AxesOverlayStrategy.updateVisualWithConfigChange(configurationUpdateEvent.TransformTree);
            end
        end

        
        function updateSelection(obj, selectionEvent)
            %updateSelection Update the highlighted object in the figure canvas

            obj.FigStateHandler.selectObject(selectionEvent, robotics.manip.internal.FigureManager.BackgroundColor);
        end
        
        function updateSceneContent(obj, sceneModelChangedEvent)
            %updateSceneContent Update the objects displayed in the scene canvas so they match those in the scene model

            obj.FigStateHandler.updateSceneContent(sceneModelChangedEvent);
        end

        function updateCollisionState(obj, collisionEventData)
            %updateCollisionState Update collision state of objects in the scene
            %   Objects in collision are highlighted in red. This method
            %   selects all the objects in collision and applies the right
            %   highlighting.

            obj.FigStateHandler.clearBodyFaceHighlights;
            if collisionEventData.RBTCollisionState == robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated
                return;
            end

            % If collision has been evaluated, apply the new collision data
            sceneObjectsMap = collisionEventData.SceneObjectsMap;
            objKeys = sceneObjectsMap.keys;
            for i = 1:sceneObjectsMap.Count
                if isfield(sceneObjectsMap(objKeys{i}), 'State') && ~isempty(sceneObjectsMap(objKeys{i}).State)
                    obj.FigStateHandler.highlightBodyFace(objKeys{i}, [1 0 0]);
                end
            end
        end

        function updateMarkerPose(obj, markerPose)
            %updateMarkerPose Set the pose of the marker in the canvas

            obj.Marker.updateMarker(markerPose);
        end

        function hideMarker(obj)
            %hideMarker Hide the marker from view

            obj.Marker.Visible = false;
        end

        function showMarker(obj)
            %showMarker Make the marker visible in the view

            obj.Marker.Visible = true;
        end

        function toggleMarkerView(obj)
            %toggleMarkerView Toggle the visibility of the marker in the view

            if obj.Marker.Visible
                obj.hideMarker();
            else
                obj.showMarker();
            end
        end

        function mpose = get.MarkerPose(obj)
            %get.MarkerPose Getter for marker pose (extracted from the marker properties)

            mpose = obj.Marker.MarkerPose;
        end
    end

    methods (Access = ?robotics.manip.internal.InteractiveMarker)
        function markerButtonUpCB(obj, markerPose)
            %markerButtonUpCB Callback for the marker mouse-button release
            %   This callback is triggered when the mouse is released on
            %   the IK control marker. It sends an event message to the
            %   appropriate listener with the current marker location.

            eventData = robotics.ikdesigner.internal.event.PoseEventData(markerPose);
            notify(obj, "MarkerButtonUp", eventData); 
        end
        
        function markerButtonMoveCB(obj, ~)
            %markerButtonMoveCB Callback to indicate the mouse has moved while the marker is selected
            %   This callback is triggered when the IK control marker is
            %   selected and the mouse is moved.

            notify(obj, "MarkerButtonMove"); 
        end
    end
end

