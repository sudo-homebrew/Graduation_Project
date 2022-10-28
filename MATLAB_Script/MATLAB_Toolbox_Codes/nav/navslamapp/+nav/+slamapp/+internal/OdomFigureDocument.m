classdef OdomFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%ODOMFIGUREDOCUMENT Figure document for display of the odom plot

% Copyright 2018-2020 The MathWorks, Inc.


    properties
        %OdomLineObject
        OdomLineObject

    end

    methods
        function obj = OdomFigureDocument(tag)
        %OdomFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.Axes = axes(obj.Figure, 'Box', 'on', 'Units', 'normalized');
            grid(obj.Axes, 'on');
            xlabel(obj.Axes, 'X');
            ylabel(obj.Axes, 'Y');
            obj.Axes.Toolbar.Visible = 'on';

            obj.SensorTransform = hgtransform('Parent', obj.Axes);
            vertPos = obj.SensorTriangleVertices;
            obj.SensorHandle = fill(obj.SensorTransform, vertPos(1,:).', vertPos(2,:).', 'w', 'FaceColor', 'none', 'LineWidth', 1, 'Visible', 'off');

            % invisible triangle, to prevent the axes from over-shrinking
            fill(obj.SensorTransform, 3*vertPos(1,:).', 3*vertPos(2,:).', 'w', 'FaceColor', 'none', 'EdgeColor', 'none', 'Visible', 'on')

            addlistener(obj.Axes, 'SizeChanged', @(src, evt) obj.rescaleSensorSize);
            addlistener(obj.Axes, 'YLim', 'PostSet', @(src, evt) obj.rescaleSensorSize); %YLim is observable
            addlistener(obj.Axes, 'XLim', 'PostSet', @(src, evt) obj.rescaleSensorSize); %XLim is observable

            % workaround to deal with the restore button on floating
            % toolbar
            addlistener(obj.Axes, 'XLimMode', 'PostSet', @(src, evt) obj.rescaleSensorSize); %XLimMode is observable
            addlistener(obj.Axes, 'YLimMode', 'PostSet', @(src, evt) obj.rescaleSensorSize); %YLimMode is observable
        end

        function drawOdom(obj, odomData)
        %drawOdom
            if (size(odomData{1}, 1) == 4)
                Ts = odomData;
                poses = zeros(numel(Ts), 3);
                for i = 1:numel(Ts)
                    T = Ts{i};
                    p = robotics.core.internal.SEHelpers.tformToPoseSE3(T);
                    ypr = quat2eul(p(4:end));
                    poses(i,:) = [p(1), p(2), ypr(1)];
                end
            else
                poses = cell2mat(odomData(:)); % make sure it's N x 1 cell
            end


            if ~isempty(obj.OdomLineObject) && isvalid(obj.OdomLineObject)
                set(obj.OdomLineObject, 'Xdata', poses(:,1), 'Ydata', poses(:,2));
            else
                hold(obj.Axes, 'on');
                obj.OdomLineObject = plot(obj.Axes, poses(:,1), poses(:,2), 'b.-');
                hold(obj.Axes, 'off');
                rescaleSensorSize(obj);
            end

            xlabel(obj.Axes, 'X');
            ylabel(obj.Axes, 'Y');

            obj.Axes.DataAspectRatioMode = 'manual';
            obj.Axes.DataAspectRatio = [1 1 1];

            obj.SensorHandle.Visible = 'on';
            obj.OdomLineObject.Visible = 'on';
        end

        function clearOdom(obj)
        %clearOdom
            if ~isempty(obj.SensorHandle) && isvalid(obj.SensorHandle)
                obj.SensorHandle.Visible = 'off';
            end

            if ~isempty(obj.OdomLineObject) && isvalid(obj.OdomLineObject)
                set(obj.OdomLineObject, 'Xdata', 0, 'Ydata', 0);
                obj.OdomLineObject.Visible = 'off';
            end
        end


    end
end
