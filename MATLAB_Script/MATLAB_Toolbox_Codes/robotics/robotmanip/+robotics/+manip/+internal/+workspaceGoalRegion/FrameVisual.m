classdef FrameVisual
%This class is for internal use only. It may be removed in the future.

%FrameVisual A set of graphics line objects that visualizes a cartesian frame

%   Copyright 2020 The MathWorks, Inc.
%#codegen

    properties
        %Name Name of the frame
        Name='Frame'

        %Pose Pose of the frame in the world frame
        Pose=eye(4)

        %Size Size is a vector of 3 elements for x, y, and z axis size
        FrameSize=...
            robotics.manip.internal.workspaceGoalRegion.WGRVisual.DefaultFrameSize

        %Size Size is a vector of 3 elements for x, y, and z axis size
        FrameWidth=[3, 3, 3]

        %FrameColor A vector of 3 elements for color of each axes
        FrameColor= {'r', 'g', 'b'}

        %AxisSpec A char vector defining the line spec of the axis
        AxisSpec = '-';
    end

    properties(Access=private, Constant)
        %Tag
        Tag = 'DO_NOT_EDIT_Frame';
    end

    methods(Access=private)
        function axisVisual = plotAxis(obj, axisIdx, parent)
        %plotAxis Creates a visual for an axis of the cartesian frame
            localPose = eye(4);
            frameSize = obj.FrameSize(axisIdx);
            frameAxis = localPose(:, axisIdx) * frameSize;
            axisVisual = plot3([0, frameAxis(1)], ...
                               [0, frameAxis(2)], ...
                               [0, frameAxis(3)], ...
                               obj.AxisSpec,...
                               'Color', obj.FrameColor{axisIdx}, ...
                               'LineWidth', obj.FrameWidth(axisIdx), ...
                               'Parent', parent);
            axisVisual.Tag = sprintf("%d Idx Axis", axisIdx);
            axisVisual.DataTipTemplate.DataTipRows = dataTipTextRow(obj.Name, "");
        end
    end

    methods
        function t = show(obj, parent)
        %show Shows the cartesian frame and attaches it to an hgtransform
            t = hgtransform("Parent", parent);
            plotAxis(obj, 1, t); % X axis
            plotAxis(obj, 2, t); % Y axis
            plotAxis(obj, 3, t); % Z axis

            % Set the pose of the frame
            set(t, 'Matrix', obj.Pose);

            t.Tag = sprintf('%s_%s', obj.Tag, obj.Name);
        end
    end
end
