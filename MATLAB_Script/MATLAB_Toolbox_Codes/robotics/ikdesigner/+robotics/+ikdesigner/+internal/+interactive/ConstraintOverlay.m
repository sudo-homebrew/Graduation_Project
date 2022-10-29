classdef ConstraintOverlay < handle
%This class is for internal use only. It may be removed in the future.

%ConstraintOverlay Constrain overlay superclass

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = protected)

        %FigureStateHandler Handle to the figure state handler object
        FigureStateHandler

        %Axes Handle to the figure axes
        Axes

        %OverlayAnnotationTextbox Annotation object used to create the legend box on the overlay
        %   The overlay legend is displayed when the overlay is active and
        %   indicates all the key visual pieces. When the overlay is
        %   removed, the legend is removed as well.
        OverlayAnnotationTextbox
    end

    properties (Access = protected, Constant)
        %REFBODYCOLOR Highlight color for the reference body (blue)
        REFBODYCOLOR = [0 0 1]

        %EEBODYCOLOR Highlight color for the end effector body (green)
        EEBODYCOLOR = [0 1 0]

        %REFBODYLEGENDTEXT Legend entry for the end effector body
        REFBODYLEGENDTEXT = string(message('robotics:ikdesigner:constrainttabviews:ReferenceBodyLegendText'))

        %EEBODYLEGENDTEXT Legend entry for the end effector body
        EEBODYLEGENDTEXT = string(message('robotics:ikdesigner:constrainttabviews:EEBodyLegendText'))

        %DEFAULTFRAMESIZE Default frame size for marker pose frame visuals
        %   The size corresponds to the length of each axes line
        DEFAULTFRAMESIZE = [0.25 0.25 0.25]
    end

    methods (Abstract)
        updateVisualWithConfigChange(obj, tformtree)
        %updateVisualWithConfigChange Update the display given a change in the configuration of the associated rigid body tree

        update(obj, constraintEvent)
        %update Update the constraint visuals
    end

    methods (Abstract, Access = protected)
        defineLegendEntriesArray(obj)
        %defineLegendEntriesArray Define the entries in the legend textbox
    end

    methods
        function delete(obj)
        %delete Destructor
        %   This custom destructor ensures that visuals associated with
        %   the constraint are cleared when the constraint setup is no
        %   longer active.

            obj.FigureStateHandler.removeHighlightColor(obj.REFBODYCOLOR);
            obj.FigureStateHandler.removeHighlightColor(obj.EEBODYCOLOR);
            delete(obj.OverlayAnnotationTextbox);
        end
    end

    methods (Access = protected)
        function updateBodySelection(obj, updateType, aimingConstraintEvent)
        %updateBodySelection Update body selection visuals
        %   This method selects bodies in pre-defined colors as part of
        %   the constraint visuals. The selected bodies may correspond
        %   to the end effector or reference body in the constraint.

            switch updateType
              case robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.RefBodySelection
                selectionColor = obj.REFBODYCOLOR;
                selectedBodyKey = aimingConstraintEvent.RefBodyKey;

              case robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.EEBodySelection
                selectionColor = obj.EEBODYCOLOR;
                selectedBodyKey = aimingConstraintEvent.EEBodyKey;
              otherwise
                return;
            end

            selectionEvent = robotics.ikdesigner.internal.event.ViewSelectionEventData(selectedBodyKey, selectedBodyKey);
            obj.FigureStateHandler.selectObject(selectionEvent, selectionColor);
        end

        function overlayLegend = drawLegend (obj, axes)
        %drawLegend Create a legend for the aiming constraint
        %   This method draws a legend on the figure that contains the
        %   provided axes. The legend is drawn as a text box annotation
        %   with three entries: colors to indicate the reference and
        %   end effector bodies, and the target point marker. The
        %   legend should exist in all overlays.

            parentFigure = ancestor(axes, 'figure');

            % Define the three legend entry strings
            legendEntries = obj.defineLegendEntriesArray();

            % Position the legend so that it is persistently in the top
            % left corner of the figure (independent of the axes zoom),
            % away from any other display like the figure tools.
            overlayPosition = [.035 .85 .1 .1];

            % Create the legend using a textbox annotation
            overlayLegend = annotation(parentFigure, 'TextBox', ...
                                       'Position', overlayPosition, ...
                                       'BackgroundColor', 'white', ...
                                       'EdgeColor', 'black', ...
                                       'FontSize', 14, ....
                                       'String', legendEntries);
        end
    end

    methods (Static, Access = protected)
        function worldTgtTform = mapLocalFrameToWorldFrame(localPose, tformTree, refBodyIdx)
        %mapLocalFrameToWorldFrame Get the position of the target frame in the world frame
        %   The target point is only ever stored on the object in a
        %   local frame relative to the origin of a specified reference
        %   body (which may change). This method is used to get the
        %   position of the target point in the global frame, using the
        %   currently specified reference body index & transform tree
        %   of the robot to indicate state.

        % Get the pose of the reference body origin in the world frame
            if refBodyIdx > 0
                worldToRefFrame = tformTree{refBodyIdx};
            else
                worldToRefFrame = eye(4);
            end

            % Compute the position of the target point in the world frame
            worldTgtTform = worldToRefFrame*localPose;
        end
    end
end
