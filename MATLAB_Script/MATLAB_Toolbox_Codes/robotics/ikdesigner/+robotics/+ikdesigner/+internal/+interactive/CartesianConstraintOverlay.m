classdef CartesianConstraintOverlay < robotics.ikdesigner.internal.interactive.ConstraintOverlay
%This class is for internal use only. It may be removed in the future.

%CartesianConstraintOverlay Class to add an cartesian constraint overlay to the scene canvas

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)

        %TargetMarkerVisual Visual marker associated with the target transform
        TargetMarkerVisual

        %BoundsMarkerVisual Visual marker associated with the target bounds
        BoundsMarkerVisual

        %TargetMarkerHGTform HG Transform whose child is the target marker
        TargetMarkerHGTform

        %BoundsMarkerHGTform HG Transform whose child is the target bounds marker
        BoundsMarkerHGTform

        %LocalTargetPose Target point in the frame of the reference body origin
        %   This property defines the position of the point that is plotted
        %   in the local frame of the reference body. It is necessary to
        %   store this on the object so that it doesn't need to be queried
        %   from the model every time the rigidbodytree configuration is
        %   updated.
        LocalTargetPose

        %RefBodyIdx The body index of the reference body
        %   The reference body index provides the index that is used to get
        %   the transform of the body that the point is defined with
        %   respect to. It is necessary to store this on the object so that
        %   it doesn't need to be newly queried from the model every time
        %   the rigidbodytree configuration is updated.
        RefBodyIdx
    end

    methods
        function obj = CartesianConstraintOverlay(constraintEvent, figStateHandler)
        %CartesianConstraintOverlay Constructor

            obj.FigureStateHandler = figStateHandler;
            obj.Axes = figStateHandler.Axes;
            obj.LocalTargetPose = constraintEvent.TargetTransform;

            % Get the body index from the scene objects map
            obj.RefBodyIdx = obj.FigureStateHandler.SceneObjectsMap(constraintEvent.RefBodyKey).BodyIndex;

            % Assign a figure state handler overlay strategy for the data
            obj.TargetMarkerVisual = robotics.manip.internal.workspaceGoalRegion.FrameVisual;
            obj.BoundsMarkerVisual = robotics.manip.internal.workspaceGoalRegion.TranslationBoundsVisual;
            [obj.TargetMarkerHGTform, obj.BoundsMarkerHGTform] = obj.drawOverlayMarkers(obj.Axes, constraintEvent.TFTree, constraintEvent.Bounds);

            % Add a legend
            obj.OverlayAnnotationTextbox = obj.drawLegend(obj.Axes);

            % Handle body selection
            obj.updateBodySelection(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.RefBodySelection, constraintEvent);
            obj.updateBodySelection(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.EEBodySelection, constraintEvent);
        end

        function updateVisualWithConfigChange(obj, tformTree)
        %updateVisualWithConfigChange Update the display given a change in the configuration of the associated rigid body tree

        % Map the target transform to a world frame
            targetFrameInWorldFrame = obj.mapLocalFrameToWorldFrame(obj.LocalTargetPose, tformTree, obj.RefBodyIdx);

            % Update the marker poses
            obj.TargetMarkerHGTform.Matrix = targetFrameInWorldFrame;
            obj.BoundsMarkerHGTform.Matrix = targetFrameInWorldFrame;

        end

        function update(obj, constraintEvent)
        %update Update the constraint visuals
        %   This method updates the constraint visuals using an
        %   eventdata source as the input. The update is split into two
        %   parts: first, the axes overlay, which specifies plot
        %   visuals, is updated. Next, the body selection is updated,
        %   if relevant.

        % Update the stored data members
            obj.LocalTargetPose =  constraintEvent.TargetTransform;
            obj.RefBodyIdx = obj.FigureStateHandler.SceneObjectsMap(constraintEvent.RefBodyKey).BodyIndex;

            obj.updateBoundsMarker(constraintEvent.Bounds);

            % Update the target point marker pose
            obj.updateVisualWithConfigChange(constraintEvent.TFTree);

            % For performance reasons, the selected bodies are only updated
            % when they are specifically indicated to have changed
            obj.updateBodySelection(constraintEvent.UpdateType, constraintEvent);
        end

        function delete(obj)
        %delete Destructor
        %   This custom destructor ensures that visuals associated with
        %   the constraint are cleared when the constraint setup is no
        %   longer active.

        % Remove the overlay before removing marker objects to avoid
        % actions on deleted handles
            delete@robotics.ikdesigner.internal.interactive.ConstraintOverlay(obj);
            delete(obj.TargetMarkerHGTform);
            delete(obj.BoundsMarkerHGTform);
            obj.TargetMarkerVisual = [];
        end
    end

    methods (Access = protected)
        function legendEntryCellArray = defineLegendEntriesArray(obj)
        %defineLegendEntriesArray Define the entries in the overlay legend
        %   Create a cell array of strings that will be displayed in
        %   the legend for the overlay.

        % Define the three legend entry strings
            refBodyLegendEntry = sprintf('{\\color[rgb]{%f, %f, %f}%s} %s', obj.REFBODYCOLOR, '\mid\mid\mid', obj.REFBODYLEGENDTEXT);
            eeBodyLegendEntry = sprintf('{\\color[rgb]{%f, %f, %f}%s} %s', obj.EEBODYCOLOR, '\mid\mid\mid', obj.EEBODYLEGENDTEXT);
            legendEntryCellArray = {refBodyLegendEntry, eeBodyLegendEntry};

        end
    end

    methods (Access = private)
        function [targetMarkerHGTF, boundsMarkerHGTF] = drawOverlayMarkers(obj, ax, tformTree, bounds)
        %drawOverlayMarkers Draw target transform and bounds markers

        % Compute the target point in a global frame and plot that on
        % top of the existing scene canvas axes
            targetFrameInWorldFrame = obj.mapLocalFrameToWorldFrame(obj.LocalTargetPose, tformTree, obj.RefBodyIdx);

            % Assign marker properties
            obj.TargetMarkerVisual.Pose = targetFrameInWorldFrame;
            obj.BoundsMarkerVisual.Pose = targetFrameInWorldFrame;
            obj.updateBounds(bounds);
            obj.TargetMarkerVisual.FrameSize = obj.DEFAULTFRAMESIZE;

            % Display the markers
            targetMarkerHGTF = obj.TargetMarkerVisual.show(ax);
            boundsMarkerHGTF = obj.BoundsMarkerVisual.show(ax);
        end

        function updateBoundsMarker(obj, constraintBounds)
        %updateBoundsMarker Update the marker that indicates bounds
        %   The bounds define a prism that is the range in which the
        %   selected end effector body may lie. This method accepts a
        %   3x2 matrix that defines the limits of this box, checks
        %   whether that matrix differs from the one currently in
        %   place, and if it differs, it updates the marker so the new
        %   bounds are visually represented.

            currentBoundsMarker = obj.BoundsMarkerHGTform.Children;
            currentBounds = [currentBoundsMarker.XData([1 2]); currentBoundsMarker.YData([1 3]); currentBoundsMarker.ZData([1 7])];

            % Check if the bounds have changed. If they have, delete the
            % current marker by deleting its parent transform and plot a
            % new one by calling show() again.
            if any(currentBounds ~= constraintBounds, 'all')
                ax = ancestor(obj.BoundsMarkerHGTform, 'axes');
                obj.BoundsMarkerHGTform.delete();
                obj.updateBounds(constraintBounds);
                obj.BoundsMarkerHGTform = obj.BoundsMarkerVisual.show(ax);
            end

        end

        function updateBounds(obj, bounds)
        %updateBounds Update the bounds marker properties from data
        %   The bounds are provided from the constraint as a 3x2 matrix
        %   of lower and upper bound along the frame X, Y, and Z axes.
        %   This method translates that data to the bounds marker,
        %   which uses properties for the range of translation along
        %   each of these axes.

            obj.BoundsMarkerVisual.TrX = bounds(1,:);
            obj.BoundsMarkerVisual.TrY = bounds(2,:);
            obj.BoundsMarkerVisual.TrZ = bounds(3,:);
        end
    end
end
