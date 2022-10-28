classdef AimingConstraintOverlay < robotics.ikdesigner.internal.interactive.ConstraintOverlay
%This class is for internal use only. It may be removed in the future.

%AimingConstraintOverlay Class to add an aiming constraint overlay to the scene canvas

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)

        %TargetPointMarker Visual marker associated with the target point
        TargetPointMarker

        %LocalTargetPoint Target point in the frame of the reference body origin
        %   This property defines the position of the point that is plotted
        %   in the local frame of the reference body. It is necessary to
        %   store this on the object so that it doesn't need to be queried
        %   from the model every time the rigidbodytree configuration is
        %   updated.
        LocalTargetPoint

        %RefBodyIdx The body index of the reference body
        %   The reference body index provides the index that is used to get
        %   the transform of the body that the point is defined with
        %   respect to. It is necessary to store this on the object so that
        %   it doesn't need to be newly queried from the model every time
        %   the rigidbodytree configuration is updated.
        RefBodyIdx
    end

    properties (Access = private, Constant)
        MARKERCOLOR = [1 0 0]

        MARKERSIZE = 20

        MARKERLINEWIDTH = 2

        TARGETPOINTLEGENDTEXT = string(message('robotics:ikdesigner:constrainttabviews:TargetPointLegendText'))
    end

    methods
        function obj = AimingConstraintOverlay(aimingConstraintEvent, figStateHandler)
        %AimingConstraintHandler Constructor

            obj.FigureStateHandler = figStateHandler;
            obj.Axes = figStateHandler.Axes;
            obj.LocalTargetPoint = aimingConstraintEvent.TargetPoint;

            % Get the body index from the scene objects map
            obj.RefBodyIdx = obj.FigureStateHandler.SceneObjectsMap(aimingConstraintEvent.RefBodyKey).BodyIndex;

            % Assign a figure state handler overlay strategy for the data
            obj.TargetPointMarker = obj.drawTargetPoint(obj.Axes, aimingConstraintEvent.TFTree);

            % Add a legend
            obj.OverlayAnnotationTextbox = obj.drawLegend(obj.Axes);

            % Handle body selection
            obj.updateBodySelection(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.RefBodySelection, aimingConstraintEvent);
            obj.updateBodySelection(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.EEBodySelection, aimingConstraintEvent);
        end

        function updateVisualWithConfigChange(obj, tformTree)
        %updateVisualWithConfigChange Update the display given a change in the configuration of the associated rigid body tree

        % Compute the target point in a global frame and plot that on
        % top of the existing scene canvas axes
            targetPtInWorldFrame = obj.maplocalTargetPtToWorldFrame(tformTree);

            % Update the marker pose
            obj.TargetPointMarker.XData = targetPtInWorldFrame(1);
            obj.TargetPointMarker.YData = targetPtInWorldFrame(2);
            obj.TargetPointMarker.ZData = targetPtInWorldFrame(3);

        end

        function update(obj, aimingConstraintEvent)
        %update Update the constraint visuals
        %   This method updates the constraint visuals using an
        %   eventdata source as the input. The update is split into two
        %   parts: first, the axes overlay, which specifies plot
        %   visuals, is updated. Next, the body selection is updated,
        %   if relevant.

        % Update the stored data members
            obj.LocalTargetPoint =  aimingConstraintEvent.TargetPoint;
            obj.RefBodyIdx = obj.FigureStateHandler.SceneObjectsMap(aimingConstraintEvent.RefBodyKey).BodyIndex;

            % Update the target point marker pose
            obj.updateVisualWithConfigChange(aimingConstraintEvent.TFTree);

            % For performance reasons, the selected bodies are only updated
            % when they are specifically indicated to have changed
            obj.updateBodySelection(aimingConstraintEvent.UpdateType, aimingConstraintEvent);
        end

        function delete(obj)
        %delete Destructor
        %   This custom destructor ensures that visuals associated with
        %   the constraint are cleared when the constraint setup is no
        %   longer active.

        % Remove the overlay before removing marker objects to avoid
        % actions on deleted handles
            delete@robotics.ikdesigner.internal.interactive.ConstraintOverlay(obj);
            delete(obj.TargetPointMarker);
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
            targetPointLegendEntry = sprintf('{\\color[rgb]{%f, %f, %f}%s} %s', obj.MARKERCOLOR, 'X', obj.TARGETPOINTLEGENDTEXT);
            legendEntryCellArray = {refBodyLegendEntry, eeBodyLegendEntry, targetPointLegendEntry};

        end
    end

    methods (Access = private)
        function targetPointMarker = drawTargetPoint(obj, ax, tformTree)
        %drawTargetPoint Draw a target point marker

        % Compute the target point in a global frame and plot that on
        % top of the existing scene canvas axes
            targetPtInWorldFrame = obj.maplocalTargetPtToWorldFrame(tformTree);

            % Plot the marker and update the visual properties
            targetPointMarker = plot3(ax, targetPtInWorldFrame(1), targetPtInWorldFrame(2), targetPtInWorldFrame(3), 'X');
            targetPointMarker.LineWidth = obj.MARKERLINEWIDTH;
            targetPointMarker.MarkerSize = obj.MARKERSIZE;
            targetPointMarker.MarkerEdgeColor = obj.MARKERCOLOR;
        end

        function targetPtInWorldFrame = maplocalTargetPtToWorldFrame(obj, tformTree)
        %maplocalTargetPtToWorldFrame Map the local target point to the world reference frame

            localTform = [eye(3) obj.LocalTargetPoint(:); 0 0 0 1];
            worldTform = obj.mapLocalFrameToWorldFrame(localTform, tformTree, obj.RefBodyIdx);
            targetPtInWorldFrame = worldTform(1:3,4);

        end
    end
end
