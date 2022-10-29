classdef IncrementalFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%INCREMENTALFIGUREDOCUMENT Figure document for display of incremental
%   scan matching

% Copyright 2018-2021 The MathWorks, Inc.


    properties


    end

    methods
        function obj = IncrementalFigureDocument(tag)
        %IncrementalFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.Axes = axes(obj.Figure, 'Box', 'on', 'Units', 'normalized');
            obj.Axes.Visible = 'off';

            grid(obj.Axes, 'on');
            obj.Axes.DataAspectRatioMode = 'manual';
            obj.Axes.DataAspectRatio = [1 1 1];
            obj.Axes.PlotBoxAspectRatioMode = 'manual';
            obj.Axes.PlotBoxAspectRatio = [1 1 1];
            obj.Axes.Toolbar.Visible = 'on';

            view(obj.Axes, -90, 90);
            xlabel(obj.Axes, 'X');
            ylabel(obj.Axes, 'Y');

            hold(obj.Axes, 'on');
            obj.RefScanTransform = hgtransform('Parent', obj.Axes);
            obj.RefScanLineObj = plot(obj.RefScanTransform, 0,0,'.', 'color', obj.RefScanColor);
            plot(obj.RefScanTransform, 0, 0, '.', 'Color', obj.RefScanColor, 'MarkerSize', 2);

            obj.RefScanXAxisLineObj = plot(obj.RefScanTransform, [0 1], [0 0], 'Color', 'k');
            obj.RefScanXAxisLineObj.Visible = 'off';

            obj.CurrentScanTransform = hgtransform('Parent', obj.Axes);
            obj.CurrentScanLineObj = plot(obj.CurrentScanTransform, 0,0, '.', 'Color', obj.CurrentScanColor);
            plot(obj.CurrentScanTransform, 0,0, '.', 'Color', obj.CurrentScanColor, 'MarkerSize', 2);
            obj.CurrentScanXAxisLineObj = plot(obj.CurrentScanTransform, [0 1], [0 0], 'Color', obj.CurrentScanColor);
            obj.CurrentScanXAxisLineObj.Visible = 'off';

            obj.XYConnectorLineObj = plot(obj.Axes, 0, 0, 'Color', 'k');

            obj.clearScanPair();
            hold(obj.Axes, 'off');

            x = imread(fullfile(obj.IconDir, 'Modify_Incremental_24px.jpg'));
            obj.BadgeModify = uibutton(obj.Figure, 'Position', [2 2 24 24], 'Icon', x, 'Text','');
            obj.BadgeModify.Visible = 'off';
        end

        function show(obj, vis)
        %show
            if nargin == 1
                obj.Axes.Visible = 'on';
            else
                obj.Axes.Visible = vis;
            end
        end
    end
end
