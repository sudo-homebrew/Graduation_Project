classdef LoopClosureFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%LOOPCLOSUREFIGUREDOCUMENT Figure document for display of scan map or occupancy
%   grid map

% Copyright 2018-2021 The MathWorks, Inc.

    properties (Constant)
        %DefaultIconButtonSize
        DefaultIconButtonSize = [24, 24];
    end

    properties

        %PrevLoopClosureStepper
        PrevLoopClosureStepper

        %NextLoopClosureStepper
        NextLoopClosureStepper
    end

    methods
        function obj = LoopClosureFigureDocument(tag)
        %LoopClosureFigureDocument Constructor
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
            obj.CurrentScanLineObj = plot(obj.CurrentScanTransform, 0,0, '.', 'color', obj.CurrentScanColor);
            plot(obj.CurrentScanTransform, 0,0, '.', 'Color', obj.CurrentScanColor, 'MarkerSize', 2);

            obj.CurrentScanXAxisLineObj = plot(obj.CurrentScanTransform, [0 1], [0 0], 'Color', obj.CurrentScanColor);
            obj.CurrentScanXAxisLineObj.Visible = 'off';

            obj.XYConnectorLineObj = plot(obj.Axes, 0, 0, 'Color', 'k');

            obj.clearScanPair();
            hold(obj.Axes, 'off');

             x = imread(fullfile(obj.IconDir, 'Modify_LoopClosure_24px.jpg'));
             obj.BadgeModify = uibutton(obj.Figure, 'Position', [2 2 24 24], 'Icon', x, 'Text', '');
             obj.BadgeModify.Visible = 'off';

            % creating backward and forward stepper buttons
            params = [];
            hspace = 5;
            params.Image = fullfile(matlabroot, 'toolbox', 'shared', 'robotics', 'robotappscore', 'icons', 'BackwardStepper_24px.png');
            axPos = obj.getAxesActualPlotBoxCoordinates;
            ibSize = obj.DefaultIconButtonSize;
            params.Position = [axPos(1)+axPos(3)+ hspace, axPos(2) + axPos(4) - ibSize(2), ibSize(1), ibSize(2)];
            params.BackgroundColor = obj.Figure.Color;
            params.Enable = 'on';
            params.TooltipString = obj.retrieveMsg('PrevLoopClosureStepperDescription');

            obj.PrevLoopClosureStepper = robotics.appscore.internal.createIconButton(obj.Figure, [obj.Figure.Tag '_PrevLoopClosureStepper'], params);
            obj.PrevLoopClosureStepper.Visible = 'off';


            params.Image = fullfile(matlabroot, 'toolbox', 'shared', 'robotics', 'robotappscore', 'icons', 'ForwardStepper_24px.png');
            params.Position = [axPos(1)+axPos(3)+ hspace, axPos(2) + axPos(4) - 2*ibSize(2) - hspace, ibSize(1), ibSize(2)];
            params.TooltipString = obj.retrieveMsg('NextLoopClosureStepperDescription');
            obj.NextLoopClosureStepper = robotics.appscore.internal.createIconButton(obj.Figure, [obj.Figure.Tag '_NextLoopClosureStepper'], params);
            obj.NextLoopClosureStepper.Visible = 'off';

            addlistener(obj.Figure, 'SizeChanged', @(src, evt) obj.repositionIconButton(hspace));
        end

        function show(obj, vis)
        %show
            if nargin == 1
                obj.Axes.Visible = 'on';
            else
                obj.Axes.Visible = vis;
            end
        end

        function repositionIconButton(obj, hspace)
        %repositionIconButton
            axPos = getAxesActualPlotBoxCoordinates(obj);
            ibSize = obj.DefaultIconButtonSize;
            pos = [axPos(1)+axPos(3)+ hspace, axPos(2) + axPos(4) - ibSize(2), ibSize(1), ibSize(2)];
            obj.PrevLoopClosureStepper.Position = pos;

            pos = [axPos(1)+axPos(3)+ hspace, axPos(2) + axPos(4) - 2*ibSize(2) - hspace, ibSize(1), ibSize(2)];
            obj.NextLoopClosureStepper.Position = pos;

        end

        function restoreToInitState(obj)
        %restoreToInitState

            restoreToInitState@nav.slamapp.internal.FigureDocument(obj);
            obj.PrevLoopClosureStepper.Visible = 'off';
            obj.NextLoopClosureStepper.Visible = 'off';
        end

        function drawScanPair(obj, refScan, currScan, relPose, scanIdPair)
        %drawScanPair

            drawScanPair@nav.slamapp.internal.FigureDocument(obj, refScan, currScan, relPose, scanIdPair);

            obj.PrevLoopClosureStepper.Visible = 'on';
            obj.NextLoopClosureStepper.Visible = 'on';

        end
    end
end
