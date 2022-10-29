classdef MapFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%MAPFIGUREDOCUMENT Figure document for display of scan map or occupancy
%   grid map

% Copyright 2018-2021 The MathWorks, Inc.

    properties

        %Slider The map figure slider
        Slider

        %ScanHGTransforms
        ScanHGTransforms

        %TrajectoryLineObj
        TrajectoryLineObj

        %LoopClosureLineObj
        LoopClosureLineObj
        
        %Poses
        Poses
        
        %VisibleHG
        VisibleHG
        
        %SensorResizeFreq
        SensorResizeFreq
        
        %NumScansSinceLastSensorResize
        NumScansSinceLastSensorResize
    end

    methods
        function obj = MapFigureDocument(tag)
        %MapFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.Slider = robotics.appscore.internal.SliderView(obj.Figure, 'MapFigSlider');

            obj.Slider.ContainerPanel.Visible = 'off';

            obj.Axes = axes(obj.Figure, 'Box', 'on', 'Units', 'pixels');
            xlabel(obj.Axes, 'X');
            ylabel(obj.Axes, 'Y');
            grid(obj.Axes, 'on');
            obj.Axes.DataAspectRatioMode = 'manual';
            obj.Axes.DataAspectRatio = [1 1 1];
            obj.Axes.PlotBoxAspectRatioMode = 'manual';
            obj.Axes.PlotBoxAspectRatio = [1 1 1];
            obj.Axes.Toolbar.Visible = 'on';
            obj.Axes.Visible = 'off';

            obj.SensorTransform = hgtransform('Parent', obj.Axes);
            vertPos = obj.SensorTriangleVertices;
            obj.SensorHandle = fill(obj.SensorTransform, vertPos(1,:).', vertPos(2,:).', 'w', 'LineWidth', 1, 'Visible', 'off');

            obj.LegendSize = [2*obj.DefaultLegendSize(1), 1.5*obj.DefaultLegendSize(2)];
            
            obj.SensorResizeFreq = 10;
            obj.NumScansSinceLastSensorResize = 0;

            % do no use SizeChangedFcn here
            addlistener(obj.Figure, 'SizeChanged', @(src, evt) obj.repositionMapAxesInFigure);
            addlistener(obj.Axes, 'YLim', 'PostSet', @(src, evt) obj.rescaleSensorSize); %YLim is observable (only triggered when user changes it)
            addlistener(obj.Axes, 'XLim', 'PostSet', @(src, evt) obj.rescaleSensorSize); %XLim is observable
        end


        function preloadScans(obj, scans)
        %preloadScans
            obj.ScanHGTransforms = {}; % clear

            hold(obj.Axes, 'on');
            for i = 1:length(scans)
                obj.ScanHGTransforms{i} = hgtransform('Parent', obj.Axes);
                cart = scans{i}.Cartesian;
                plot(obj.ScanHGTransforms{i}, cart(:,1), cart(:,2), 'm.');
                obj.ScanHGTransforms{i}.Visible = 'off';
            end
            obj.TrajectoryLineObj = plot(obj.Axes, 0, 0, 'b.-');
            obj.TrajectoryLineObj.Visible = 'off';

            obj.LoopClosureLineObj = plot(obj.Axes, 0, 0, 'r-');
            obj.LoopClosureLineObj.Visible = 'off';

            hold(obj.Axes, 'off');
            obj.Poses = zeros(3,length(scans));
            obj.VisibleHG = false(length(scans),1);
            
            obj.SensorTransform.Visible = 'on';
        end
        
        function showCurrentMap(obj, scanIDs, poses, lcEdges)
        %showCurrentMap
            
            p = poses';
            changedPoseInd = any(obj.Poses(:,1:length(scanIDs))-p);
            visibilityUpdate = false(length(obj.VisibleHG),1);
            visibilityUpdate(1:length(scanIDs)) = true;
            changedHGTransformInd = xor(visibilityUpdate,obj.VisibleHG);
            obj.clearMap(changedHGTransformInd);
            fd = find(changedPoseInd);
            obj.Poses(:,visibilityUpdate) = p;
            obj.VisibleHG = visibilityUpdate;
            
            lcXData = [];
            lcYData = [];
            for i = 1:length(fd)
                T1 = makehgtform('translate', [poses(fd(i),1), poses(fd(i),2), 0]);
                T2 = makehgtform('zrotate', poses(fd(i),3));
                T = T1*T2;
                obj.ScanHGTransforms{scanIDs(fd(i))}.Matrix = T;
            end
            T1 = makehgtform('translate', [poses(end,1), poses(end,2), 0]);
                T2 = makehgtform('zrotate', poses(end,3));
                T = T1*T2;
            TCurrent = T;

            for j = 1:size(lcEdges, 1)
                p1 = poses(lcEdges(j,1), :);
                p2 = poses(lcEdges(j,2), :);
                lcXData = [lcXData, nan, p1(1), p2(1)]; %#ok<AGROW>
                lcYData = [lcYData, nan, p1(2), p2(2)]; %#ok<AGROW>
            end
            obj.TrajectoryLineObj.Visible = 'on';
            set(obj.TrajectoryLineObj, 'XData', poses(:,1), 'YData', poses(:,2));

            obj.LoopClosureLineObj.Visible = 'on';
            set(obj.LoopClosureLineObj, 'XData', lcXData, 'YData', lcYData);
            obj.SensorHandle.Visible = 'on';
            obj.drawSensor(TCurrent);
            if mod(obj.NumScansSinceLastSensorResize,obj.SensorResizeFreq) == 0
                % sensor resize queries for y limits which is slow when a
                % lot of data is plotted on the axes. So calling reduse the
                % frequency of sensor resize
                obj.rescaleSensorSize();
                obj.NumScansSinceLastSensorResize = 0;
            end
            obj.NumScansSinceLastSensorResize = obj.NumScansSinceLastSensorResize + 1;

            if isempty(obj.Legend) || strcmp(obj.Legend.Visible, 'off')
                % render legend
                legendStrs = {...
                    obj.retrieveMsg('RobotTrajectoryLegendName'), ...
                    obj.retrieveMsg('LoopClosureLegendName'), ...
                    obj.retrieveMsg('ScansLegendName')};
%                 
                axPos = obj.getAxesActualPlotBoxCoordinates();
                pos = [axPos(1), axPos(2), obj.LegendSize(1), obj.LegendSize(2)];
                
                % Legend auto update is not needed because we are not
                % introducing new plots as we go. And also the legend auto
                % update is very slow.
                obj.Legend = legend(...
                    [obj.TrajectoryLineObj, obj.LoopClosureLineObj, obj.ScanHGTransforms{1}.Children(1)], ...
                    legendStrs, 'Units', 'pixels', 'Position', pos, 'Box', 'off', 'AutoUpdate',"off", 'Color', 'none');
            end
        end
        
        function clearMap(obj, up)
        %clearMap
            if nargin < 2
                up = false(length(obj.VisibleHG),1);
            end
            fd = find(up);
            for j = 1:length(fd)
                if strcmp(obj.ScanHGTransforms{fd(j)}.Visible , 'off')
                    obj.ScanHGTransforms{fd(j)}.Visible = 'on';
                else
                    obj.ScanHGTransforms{fd(j)}.Visible = 'off';
                end
            end
            obj.TrajectoryLineObj.Visible = 'off';
            obj.LoopClosureLineObj.Visible = 'off';
            obj.SensorHandle.Visible = 'off';
        end
        
        function show(obj, vis)
        %show
            if nargin == 1
                obj.Slider.ContainerPanel.Visible = 'on';
                obj.Axes.Visible = 'on';
            else
                obj.Slider.ContainerPanel.Visible = vis;
                obj.Axes.Visible = vis;
            end
        end


        function repositionMapAxesInFigure(obj)
        %repositionMapAxesInFigure
            obj.repositionAxesInFigure();
            obj.rescaleSensorSize();
            obj.repositionLegend();
        end

        function restoreToInitState(obj)
        %restoreToInitState

            restoreToInitState@nav.slamapp.internal.FigureDocument(obj);

            if ~isempty(obj.ScanHGTransforms)
                for i = 1:numel(obj.ScanHGTransforms)
                    if ~isempty(obj.ScanHGTransforms{i}) && isvalid(obj.ScanHGTransforms{i})
                        obj.ScanHGTransforms{i}.delete;
                    end
                end
                obj.ScanHGTransforms = {};
            end

            if ~isempty(obj.TrajectoryLineObj) && isvalid(obj.TrajectoryLineObj)
                obj.TrajectoryLineObj.delete;
            end
            obj.TrajectoryLineObj = [];

            if ~isempty(obj.LoopClosureLineObj) && isvalid(obj.LoopClosureLineObj)
                obj.LoopClosureLineObj.delete;
            end
            obj.LoopClosureLineObj = [];

            obj.SensorTransform.Visible = 'off';
            obj.Banner.Visible = 'off';

            obj.Axes.Visible = 'off';

        end
    end
end
