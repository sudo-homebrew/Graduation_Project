classdef ScanFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%SCANFIGUREDOCUMENT Figure document for display of a scan plot

% Copyright 2018-2021 The MathWorks, Inc.

    properties (Constant)
        ScanTimeTextFontSize = 10;

        ScanTimeTextColor = [0.5 0.6 0.7]; % lake blue
    end

    properties
        %ScanLineObject
        ScanLineObject

        %Slider The scan figure has a customized slider
        Slider

        %ScanTimeText
        ScanTimeText
    end

    methods
        function obj = ScanFigureDocument(tag)
        %ScanFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.Slider = robotics.appscore.internal.SliderView(obj.Figure, 'ScanFigSlider');

            obj.Axes = axes(obj.Figure, 'Box', 'on', 'Units', 'pixels');
            grid(obj.Axes, 'on');
            obj.Axes.Toolbar.Visible = 'on';

            obj.ScanTimeText = uilabel( ...
                obj.Figure, ...
                'Text', '', ...
                'HandleVisibility', 'off', ...
                'HorizontalAlignment', 'left', ...
                'FontSize', obj.ScanTimeTextFontSize, ...
                'FontWeight', 'bold', ...
                'Position', [300, 10, 200, 25], ...
                'Visible', 'on');


            % do no use SizeChangedFcn here
            addlistener(obj.Figure, 'SizeChanged', @(src, evt) obj.repositionScanAxesInFigure);

            addlistener(obj.Axes, 'YLim', 'PostSet', @(src, evt) obj.rescaleSensorSize); %YLim is observable

            % the resizing of the slider is handled by its own controller

        end

        function drawCurrentRawScan(obj, scan, maxRange, scanTime)
        %drawCurrentRawScan
            cart = scan.Cartesian;
            if ~isempty(obj.ScanLineObject) && isvalid(obj.ScanLineObject)
                set(obj.ScanLineObject, 'xdata', cart(:,1), 'ydata', cart(:,2));
            else
                obj.ScanLineObject = plot(obj.Axes, cart(:,1), cart(:,2),'m.');

                hold(obj.Axes, 'on');
                vertPos = obj.SensorTriangleVertices; %0.08*min(8, maxRange)*
                obj.SensorHandle = fill(obj.Axes, vertPos(1,:).', vertPos(2,:).', 'w', 'LineWidth',1);
                hold(obj.Axes, 'off');

                view(obj.Axes, -90, 90);
                xlabel(obj.Axes, 'X');
                ylabel(obj.Axes, 'Y');

                obj.Axes.PlotBoxAspectRatioMode = 'manual';
                obj.Axes.PlotBoxAspectRatio = [1 1 1];

                obj.Axes.DataAspectRatioMode = 'manual';
                obj.Axes.DataAspectRatio = [1 1 1];

                xlim(obj.Axes, 0.8*[-maxRange maxRange]);
                ylim(obj.Axes, 0.8*[-maxRange maxRange]);

                grid(obj.Axes, 'on');
            end

            if nargin > 3
                obj.ScanTimeText.Text = sprintf('%8.4fs', scanTime);
            else
                obj.ScanTimeText.Text = '';
            end
        end
        
        function clearScan(obj)
        %clearScan
            if ~isempty(obj.SensorHandle) && isvalid(obj.SensorHandle)
                obj.SensorHandle.Visible = 'off';
            end

            if ~isempty(obj.ScanLineObject) && isvalid(obj.ScanLineObject)
                set(obj.ScanLineObject, 'Xdata', 0, 'Ydata', 0);
                obj.ScanLineObject.Visible = 'off';
            end
        end

        function repositionScanAxesInFigure(obj)
        %repositionScanAxesInFigure
            obj.repositionAxesInFigure();
            obj.rescaleSensorSize();
        end



    end
end
