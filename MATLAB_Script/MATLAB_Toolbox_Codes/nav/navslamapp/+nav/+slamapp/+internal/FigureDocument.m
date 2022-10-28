classdef FigureDocument < handle & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper

    %This class is for internal use only. It may be removed in the future.

    %FIGUREDOCUMENT Thin wrapper for a HG figure in the app

    % Copyright 2018-2021 The MathWorks, Inc.

    properties (Constant)
        %BannerBackgroundColor
        BannerBackgroundColor = [255 255 225]./255 % beige color

        %BannerFontSize
        BannerFontSize = 10;

        %SensorTriangleVertices Represents the sensor triangle in the scan map and odometry plots
        SensorTriangleVertices = [[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]] * 0.5;

        %SensorLength
        SensorLength = 0.55;

        %RefScanColor
        RefScanColor = [0 0.4470 0.7410];

        %RefScanColorGrey
        RefScanColorGrey = [0.4470 0.4470 0.4470];

        %CurrentScanColor
        CurrentScanColor = [0.8500 0.6250 0.0980];

        %CurrentScanColorGrey
        CurrentScanColorGrey = [0.6250 0.6250 0.6250];

        %SyncColor
        SyncColor = [1, 0.5, 0];

        %DefaultLegendSize
        DefaultLegendSize = [80, 40]; % in pixels

        %IconDir
        IconDir = fullfile(matlabroot, 'toolbox', 'nav', 'navslamapp', 'icons');

    end

    properties
        %Figure The actual uifigure and child of an app container figure document
        Figure
        
        %FigureDoc figure document comprises of uifigure and it can be
        %added to app container for visualization.
        FigureDoc

        %Axes The Cartesian axes in the figure
        Axes

        %Banner
        Banner

        %InfoTex Text in the banner
        InfoText

        %BadgeModify
        BadgeModify

        %Legend
        Legend

        %LegendSize
        LegendSize
    end

    properties
        %SensorHandle
        SensorHandle

        %SensorTransform
        SensorTransform
    end

    properties
        %RefScanTransform
        RefScanTransform

        %RefScanLineObj
        RefScanLineObj

        %CurrentScanTransform
        CurrentScanTransform

        %CurrentScanLineObj
        CurrentScanLineObj

    end

    properties
        %RefScanXAxisLineObj
        RefScanXAxisLineObj

        %CurrentScanXAxisLineObj
        CurrentScanXAxisLineObj

        %XYConnectorLineObj
        XYConnectorLineObj
    end

    properties (Dependent)
        %Name Name of the figure
        Name

    end

    properties (Access = protected)
        BannerTextHeight = 16;
    end

    events
    FigureDocument_KeyboardRequest
end


methods
    function obj = FigureDocument(tag)
    %FigureDocument Constructor
        % Create ui figure document
        obj.createUIFigureDocument(tag);

        obj.Banner = uipanel( ...
            obj.Figure, ...
            'HandleVisibility', 'off', ...
            'BorderType', 'line', ...
            'Units', 'pixels', ...
            'Position', [1 1 1 1], ...
            'BackgroundColor', obj.BannerBackgroundColor, ...
            ...'HighlightColor', 'k', ... Not supported
            'Visible', 'off');
        obj.Banner.AutoResizeChildren = 'off';

        obj.InfoText = uilabel( ...
            obj.Banner, ...
            'HandleVisibility', 'off', ...
            'BackgroundColor', obj.BannerBackgroundColor, ...
            'HorizontalAlignment', 'left', ...
            'FontSize', obj.BannerFontSize, ...
            'FontWeight', 'bold', ...
            'Text', obj.retrieveMsg('ReminderToApplyChanges'), ...
            'Position', [1 1 1 1],   ...
            'Visible', 'on');

        obj.LegendSize = obj.DefaultLegendSize;

        addlistener(obj.Figure, 'SizeChanged', @(~,~)resizeFigDocElements(obj));
    end

    function resizeFigDocElements(obj)
    %resizeFigDocElements
        obj.resizeBanner();
        obj.repositionLegend();
    end

    function name = get.Name(obj)
    %get.Name
        name = obj.Figure.Name;
    end

    function resizeBanner(obj)
    %resizeBanner Invoked when the figure window resizes

        figPos = obj.Figure.Position;

        % Text gets offset automatically past border of parent panel,
        % so (1,1) is up and right of lower-left border line.

        % when displayed, the banner will span the full width of the
        % figure, regardless of text width, and the height of the banner
        % is 4 pixels larger than given text height


        bannerHeight = obj.BannerTextHeight + 4;

        panelPos = [1 figPos(4)+1-bannerHeight figPos(3) bannerHeight];

        obj.Banner.Position = panelPos;

        % The widest we can go with the text is figure width - 2:
        obj.InfoText.Position  = [1 3 figPos(3)-2 obj.BannerTextHeight];


    end

    function showBanner(obj, v)
    %showBanner

        if v > 0
            obj.Banner.Visible = 'on';
        else
            obj.Banner.Visible = 'off';
        end
    end

    function delete(obj)
    %delete
        if (~isempty(obj.Figure) && isvalid(obj.Figure))
            delete(obj.Figure);
        end
    end

    function repositionAxesInFigure(obj)
    %repositionAxesInFigure For figure documents who need manual
    %   resizing
        ax = obj.Axes;
        ax.Position(1) = 40;
        ax.Position(2) = 105;
        bottomClearance = 30; %20

        ax.Position(3) = max(300, obj.Figure.InnerPosition(3) - 2*ax.Position(1));
        ax.Position(4) = max(300, obj.Figure.InnerPosition(4) - bottomClearance - ax.Position(2));

    end

    function repositionLegend(obj)
    %repositionLegend Manually reposition the legend during figure
    %   resizing
        if ~isempty(obj.Legend) && ishandle(obj.Legend) && isvalid(obj.Legend)
            axPos = getAxesActualPlotBoxCoordinates(obj);

            pos = [axPos(1), axPos(2), obj.LegendSize(1), obj.LegendSize(2)];

            obj.Legend.Position = pos;
        end

    end

    function rescaleSensorSize(obj)
    %rescaleSensorSize
        pixelPos = getpixelposition(obj.Axes); % in case the 'units' are not 'pixels'
        ypixels = min(pixelPos(3), pixelPos(4));
        y = obj.Axes.YLim(2) - obj.Axes.YLim(1);

        desiredSensorPixels = 12;
        scale = (desiredSensorPixels/ypixels*y)/obj.SensorLength ;
        vertPos = scale*obj.SensorTriangleVertices;
        set(obj.SensorHandle, 'XData', vertPos(1,:).', 'YData', vertPos(2,:).');
    end

    function len = getLengthGivenPixels(obj, pixels)
    %getLengthGivenPixels
        pixelPos = getpixelposition(obj.Axes);
        ypixels = min(pixelPos(3), pixelPos(4));
        y = obj.Axes.YLim(2) - obj.Axes.YLim(1);

        len = pixels/ypixels*y;
    end

    function drawSensor(obj, T)
    %drawSensor

        obj.SensorTransform.Matrix = T;

    end

    function drawScanPair(obj, refScan, currScan, relPose, scanIdPair)
    %drawScanPair

        refCart = refScan.Cartesian;
        set(obj.RefScanLineObj, 'XData', refCart(:,1), 'YData', refCart(:,2));

        currCart = currScan.Cartesian;
        set(obj.CurrentScanLineObj, 'XData', currCart(:,1), 'YData', currCart(:,2));

        obj.transformCurrentScan(relPose);

        obj.RefScanTransform.Visible = 'on';
        obj.CurrentScanTransform.Visible = 'on';

        % render legend
        legendStrs = {sprintf('%s %d', obj.retrieveMsg('ScanLegendName'), scanIdPair(1)), sprintf('%s %d', obj.retrieveMsg('ScanLegendName'), scanIdPair(2))};
        if isempty(obj.Legend) || strcmp(obj.Legend.Visible, 'off')
            axPos = obj.getAxesActualPlotBoxCoordinates();
            pos = [axPos(1), axPos(2), obj.LegendSize(1), obj.LegendSize(2)];
            obj.Legend = legend([obj.RefScanLineObj, obj.CurrentScanLineObj], legendStrs, 'Units', 'pixels', 'Position', pos, 'Box', 'off','AutoUpdate',"off",'Color','none');
        else
            % only update the legend strings instead of computationally
            % extensive legend creation. axes limits are fixed for scan
            % pair plot, so position update is not required while
            % updating the scan data each time.
            obj.Legend.String = legendStrs;
        end
    end

    function transformCurrentScan(obj, relPose)
    %transformCurrentScan
        T1 = makehgtform('translate', [relPose(1), relPose(2), 0]);
        T2 = makehgtform('zrotate', relPose(3));
        T = T1*T2;
        obj.CurrentScanTransform.Matrix = T;

    end

    function clearScanPair(obj)
    %clearScanPair

        obj.RefScanTransform.Visible = 'off';
        obj.CurrentScanTransform.Visible = 'off';
        if ~isempty(obj.Legend) && ishandle(obj.Legend) && isvalid(obj.Legend)
            obj.Legend.Visible = 'off';
        end
    end

    function setXYLimits(obj, rangeLimits)
    %setXYLimits
        maxRange = rangeLimits(2);
        s = 1.2;
        xlim(obj.Axes, s*[-maxRange maxRange]);
        ylim(obj.Axes, s*[-maxRange maxRange]);

    end

    function allModesOff(obj)
    %allModesOff Turns off all the modes
        pan(obj.Axes, 'off');
        zoom(obj.Axes, 'off');
        datacursormode(obj.Figure, 'off');
        brush(obj.Figure, 'off');
    end

    function dim(obj)
    %dim
        obj.RefScanLineObj.Color = obj.RefScanColorGrey;
        obj.CurrentScanLineObj.Color = obj.CurrentScanColorGrey;
    end

    function lightUp(obj)
    %lightUp
        obj.RefScanLineObj.Color = obj.RefScanColor;
        obj.CurrentScanLineObj.Color = obj.CurrentScanColor;
    end

    function restoreToInitState(obj)
    %clearAxes
        obj.clearScanPair;
        obj.BadgeModify.Visible = 'off';
        obj.Axes.Visible = 'off';
    end

    function axPosMod = getAxesActualPlotBoxCoordinates(obj)
    %getAxesActualPlotBoxCoordinates
    %   This method is necessary when "Axis equal" is used
        axPos = getpixelposition(obj.Axes);
        if axPos(3) > axPos(4) % if landscape
            ratio = (obj.Axes.XLim(2) - obj.Axes.XLim(1))/(obj.Axes.YLim(2) - obj.Axes.YLim(1));
            axPos(1) = axPos(1) + 0.5*axPos(3) - 0.5*ratio*axPos(4);
            axPos(3) = axPos(4);
        else
            ratio = (obj.Axes.YLim(2) - obj.Axes.YLim(1))/(obj.Axes.XLim(2) - obj.Axes.XLim(1));
            axPos(2) = axPos(2) + 0.5*axPos(4) - 0.5*ratio*axPos(3);
            axPos(4) = axPos(3);
        end
        axPosMod = axPos;
    end

    function keyPressFcn(obj, src, evt)
    %keyPressFcn
        modifier = get(src,'CurrentModifier');
        if isempty(modifier)
            return
        end

        import robotics.appscore.internal.eventdata.*
        if strcmp(modifier, 'control') && (strcmp(evt.Key, 's') || strcmp(evt.Key, 'S'))
            % ctrl+s to save session
            obj.notify('FigureDocument_KeyboardRequest', VectorEventData(1));
        elseif strcmp(modifier, 'control') && (strcmp(evt.Key, 'o') || strcmp(evt.Key, 'O'))
            % ctrl+o to open session
            obj.notify('FigureDocument_KeyboardRequest', VectorEventData(0));
        end
    end
    
    function createUIFigureDocument(obj, tag)
            %createUIFigureDocument create ui figure document. Only
            %   uifigures that is contained within figure document can be
            %   used for visualization in app container. 

            obj.MsgIDPrefix = 'nav:navslamapp:figuredocument';
            figOptions.Title = obj.retrieveMsg([tag 'Name']);
            figOptions.DocumentGroupTag = tag;
            % figure close is not permitted
            figOptions.Closable = false;
            obj.FigureDoc = matlab.ui.internal.FigureDocument(figOptions);
            obj.FigureDoc.Maximizable = false;
            obj.Figure = obj.FigureDoc.Figure;
            obj.Figure.Name = figOptions.Title;
            obj.Figure.Tag = tag;

            obj.Figure.NumberTitle = 'off';
            obj.Figure.HandleVisibility = 'callback';
            obj.Figure.IntegerHandle = 'off';
            obj.Figure.MenuBar = 'none';
            obj.Figure.WindowKeyPressFcn = @obj.keyPressFcn;
            obj.Figure.AutoResizeChildren = 'off';
            brush(obj.Figure, 'on');
        end
end
end
