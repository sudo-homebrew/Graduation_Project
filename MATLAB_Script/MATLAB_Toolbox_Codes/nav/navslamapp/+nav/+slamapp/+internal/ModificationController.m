classdef ModificationController < handle
%This class is for internal use only. It may be removed in the future.

%MODIFICATIONCONTROLLER This class deals with the message passing for
%   modification tab

% Copyright 2018-2021 The MathWorks, Inc.

    properties
        AppViewOrg

        ModificationTab

        ModificationModel

        StateMachineModel

        MapBuilderModel
    end

    properties
        %FigDoc Figure document of choice
        FigDoc

        %Origin The origin of the current scan w.r.t. reference scan frame
        Origin

        %ThetaOffset The "offset angle" upon mouse click
        %   This is the angle BETWEEN the segment that connects the mouse
        %   click point and the current scan origin AND direction of x axis
        %   in reference scan frame
        ThetaOffset

        %ThetaInit The Theta value as in relative pose
        ThetaInit

        %XYOffset The XY coordinates of the mouse click location in reference scan frame
        XYOffset

        %PixelOffset The pixel coordinate of the mouse click
        PixelOffset

        %XYInit The XY values as in relative pose
        XYInit

        %PtToPxRatio The approximate ratio between the numerical length in axes and the pixel numbers at
        %   the time point of mouse click
        PtToPxRatio

        %WindowMouseMotionListener
        WindowMouseMotionListener

        %WindowMouseReleaseListener
        WindowMouseReleaseListener
    end

    methods
        function obj = ModificationController(modificationModel, stateMachineModel, mapBuilderModel, modificationTab, appViewOrg)
        %MODIFICATIONCONTROLLER Constructor
            obj.AppViewOrg = appViewOrg;
            obj.ModificationTab = modificationTab;
            obj.ModificationModel = modificationModel;
            obj.StateMachineModel = stateMachineModel;
            obj.MapBuilderModel = mapBuilderModel;

            obj.addViewListeners();
            obj.addModelListeners();
        end

    end

    methods
        function addViewListeners(obj)
        %addViewListeners

            obj.ModificationTab.XSpinner.ValueChangedFcn = @(source, event) obj.setModificationModelTentativeProp('X', event.EventData.Value);
            obj.ModificationTab.YSpinner.ValueChangedFcn = @(source, event) obj.setModificationModelTentativeProp('Y', event.EventData.Value);
            obj.ModificationTab.ThetaSpinner.ValueChangedFcn = @(source, event) obj.setModificationModelTentativeProp('Theta', event.EventData.Value);

            obj.ModificationTab.AcceptButton.ButtonPushedFcn = @(source, event) obj.acceptCallback();

            obj.ModificationTab.CancelButton.ButtonPushedFcn = @(source, event) obj.cancelCallback();

            addlistener(obj.ModificationTab, 'ModificationTab_Linear', @(source, event) obj.ModificationModel.set('Mode', 0));
            addlistener(obj.ModificationTab, 'ModificationTab_Angular', @(source, event) obj.ModificationModel.set('Mode', 1));
            addlistener(obj.ModificationTab, 'ModificationTab_Ignore', @(source, event) obj.setModificationModelTentativeProp('IsIgnored', true));
            addlistener(obj.ModificationTab, 'ModificationTab_Reset', @(source, event) obj.resetRelativePose);
        end

        function addModelListeners(obj)
        %addModelListeners

            addlistener(obj.ModificationModel, 'Tentative', 'PostSet', @(source, event) obj.updateToolstripAndFigure(event.AffectedObject.Tentative));


            addlistener(obj.ModificationModel, 'XYLimit', 'PostSet', @(source, event) obj.resetSpinnerXYLimit(event.AffectedObject.XYLimit));
            addlistener(obj.ModificationModel, 'ModificationModel_SubscribeButtonDownCallback', @(source, event) obj.assignCallback(event.Vector));
            addlistener(obj.ModificationModel, 'Mode', 'PostSet', @(source, event) obj.updateButtonsAndMousePointer(event.AffectedObject.Mode));
            addlistener(obj.ModificationModel, 'TabTitleName', 'PostSet', @(source, event) obj.setTabTitle(event.AffectedObject.TabTitleName));
        end
    end


    methods
        function updateToolstripAndFigure(obj, tentative)
        %updateToolstripAndFigure

            obj.ModificationTab.XSpinner.Value = tentative.X;
            obj.ModificationTab.YSpinner.Value = tentative.Y;
            obj.ModificationTab.ThetaSpinner.Value = tentative.Theta;
            obj.ModificationTab.updateToolstrip(obj.ModificationModel.IsIncremental, tentative.IsIgnored);
            if tentative.IsIgnored ~= obj.ModificationModel.IsIgnored || ...
                    abs(tentative.X - obj.ModificationModel.X) > 1e-6 || ...
                    abs(tentative.Y - obj.ModificationModel.Y) > 1e-6 || ...
                    abs(tentative.Theta - obj.ModificationModel.Theta) > 1e-6
                obj.ModificationTab.AcceptButton.Enabled = true;
            else
                obj.ModificationTab.AcceptButton.Enabled = false;
            end

            obj.updateCurrentScanPlot(tentative);
        end

        function setModificationModelTentativeProp(obj, propName, value)
        %setModificationModelTentativeProp
            obj.ModificationModel.Tentative.(propName) = value;

        end

        function resetRelativePose(obj)
        %resetRelativePose

            scanIdPair = obj.ModificationModel.UserFacingScanIdPair;

            relPose = [0 0 0];
            if obj.MapBuilderModel.AutoScanMatchingResults.isKey(num2str(scanIdPair))
                relPose = obj.MapBuilderModel.AutoScanMatchingResults(num2str(scanIdPair));
            end

            s.X = relPose(1);
            s.Y = relPose(2);
            s.Theta = relPose(3);

            s.IsIgnored = false;


            obj.ModificationModel.Tentative = s;
        end


        function resetSpinnerXYLimit(obj, v)
        %resetSpinnerXYLimit
            obj.ModificationTab.XSpinner.Limits = [-v, v];
            obj.ModificationTab.YSpinner.Limits = [-v, v];
        end

        function resetSpinnerThetaLimit(obj, v)
        %resetSpinnerThetaLimit
            obj.ModificationTab.ThetaSpinner.Limits = [-v, v];
        end
    end

    methods
        function acceptCallback(obj)
        %acceptCallback
            if obj.ModificationModel.Tentative.IsIgnored
                obj.MapBuilderModel.updateIgnoredEdgeBacklog(obj.ModificationModel.UserFacingScanIdPair);
            else

                % if the scan pair being restored was in ignored list (i.e. wasIgnored), make
                % sure to remove it from the ignored list
                if isempty(obj.MapBuilderModel.IgnoredEdges)
                    wasIgnored = false;
                    loc = -1;
                else
                    [wasIgnored, loc] = ismember(obj.ModificationModel.UserFacingScanIdPair, obj.MapBuilderModel.IgnoredEdges, 'row');
                end
                if wasIgnored
                    obj.MapBuilderModel.IgnoredEdges(loc, :) = [];
                end

                obj.ModificationModel.X = obj.ModificationModel.Tentative.X;
                obj.ModificationModel.Y = obj.ModificationModel.Tentative.Y;
                obj.ModificationModel.Theta = obj.ModificationModel.Tentative.Theta;

                obj.MapBuilderModel.updateManualModificationBacklog(obj.ModificationModel.UserFacingScanIdPair, ...
                                                                  [obj.ModificationModel.X, obj.ModificationModel.Y, obj.ModificationModel.Theta]);
            end


            obj.FigDoc.Figure.WindowButtonDownFcn = [];
            obj.updateButtonsAndMousePointer(2);
            obj.StateMachineModel.toPreviousState();

            obj.MapBuilderModel.IsAppDirty = true;

            import robotics.appscore.internal.eventdata.VectorEventData
            obj.ModificationModel.notify('ModificationModel_RequestRefreshMapSlider', VectorEventData(obj.ModificationModel.UserFacingScanIdPair));
        end

        function cancelCallback(obj)
        %cancelCallback

            obj.FigDoc.Figure.WindowButtonDownFcn = [];
            obj.updateButtonsAndMousePointer(2);
            obj.StateMachineModel.toPreviousState();


        end

        function updateCurrentScanPlot(obj, tentative)
        %updateCurrentScanPlot

            if isempty(obj.FigDoc)
                return;
            end

            if ~isempty(tentative.X) && ~isempty(tentative.Y) && ~isempty(tentative.Theta)
                obj.FigDoc.CurrentScanTransform.Matrix = makehgtform('translate', [tentative.X, tentative.Y, 0]) ...
                    * makehgtform('zrotate', tentative.Theta);
                drawnow;
            end

            if tentative.IsIgnored
                obj.FigDoc.dim();
            else
                obj.FigDoc.lightUp();
            end
        end

        function assignCallback(obj, isIncremental)
        %assignCallback

        % if the app is not currently in modification tab,
        % do not subscribe any callback


            if isIncremental
                %addlistener(obj.AppViewOrg.IncrementalFigDoc.Figure, 'WindowMousePress', @obj.windowMouseButtonDownCallback);

                % here we do want the zoom mode to shadow the window button down callback
                obj.FigDoc = obj.AppViewOrg.IncrementalFigDoc;
                obj.FigDoc.Figure.WindowButtonDownFcn = @obj.windowMouseButtonDownCallback;
            else
                %addlistener(obj.AppViewOrg.LoopClosureFigDoc.Figure, 'WindowMousePress', @obj.windowMouseButtonDownCallback);
                obj.FigDoc = obj.AppViewOrg.LoopClosureFigDoc;
                obj.FigDoc.Figure.WindowButtonDownFcn = @obj.windowMouseButtonDownCallback;
            end


        end

        function windowMouseButtonDownCallback(obj, ~, ~)
        %windowMouseButtonDownCallback

            

            if ~isempty(obj.ModificationModel.Tentative)

                figDoc = obj.FigDoc;
                offset = figDoc.CurrentScanTransform.Matrix*[0 0 0 1]';
                obj.Origin = offset(1:2);
                pt = figDoc.Axes.CurrentPoint(1, 1:2);
                obj.PixelOffset = figDoc.Figure.CurrentPoint;
                axPixelPos = getpixelposition(figDoc.Axes);
                obj.PtToPxRatio = (figDoc.Axes.XLim(2) - figDoc.Axes.XLim(1)) / axPixelPos(4);

                obj.XYOffset = pt;
                obj.XYInit = [obj.ModificationModel.Tentative.X, obj.ModificationModel.Tentative.Y];

                % set the mouse motion listeners right after computing xy
                % offset and xy init, so we won't miss any mouse callbacks
                % while changing mouse pointer and line connector
                % visualization options
                if ~isempty(obj.WindowMouseMotionListener) && isvalid(obj.WindowMouseMotionListener)
                    delete(obj.WindowMouseMotionListener);
                    delete(obj.WindowMouseReleaseListener);
                end
                obj.WindowMouseMotionListener = addlistener(figDoc.Figure, 'WindowMouseMotion', @obj.windowMouseMotionCallback);
                obj.WindowMouseReleaseListener = addlistener(figDoc.Figure, 'WindowMouseRelease', @obj.windowMouseReleaseCallback);
                
                if obj.ModificationModel.Mode == 0 % linear
                    figDoc.XYConnectorLineObj.Visible = 'on';
                    set(obj.FigDoc.XYConnectorLineObj, 'XData', [0 obj.Origin(1) obj.Origin(1)], 'YData', [0 0 obj.Origin(2)]);

                else % angular
                    pixels = 100;
                    len = figDoc.getLengthGivenPixels(pixels);
                    figDoc.CurrentScanXAxisLineObj.Visible = 'on';
                    figDoc.RefScanXAxisLineObj.Visible = 'on';

                    set(figDoc.CurrentScanXAxisLineObj, 'XData', [0 len], 'YData', [0 0]);
                    set(figDoc.RefScanXAxisLineObj, 'XData', [offset(1) len+offset(1)], 'YData', [offset(2) offset(2)]);

                    obj.ThetaOffset = atan2(pt(2) - obj.Origin(2), pt(1) - obj.Origin(1));
                    obj.ThetaInit = obj.ModificationModel.Tentative.Theta;
                end
            end
        end

        function windowMouseMotionCallback(obj, ~, evt)
        %windowMouseMotionCallback
            px = evt.Point;
            pt = evt.IntersectionPoint(1:2); % 1-by-3

            lx = px(2) - obj.PixelOffset(2);
            lt = pt(1) - obj.XYOffset(1);
            if abs(lt/obj.PtToPxRatio - lx) > 20 % 20 pixels, filtering to prevent sudden jump in mouse click jump readings
                return;
            end

            if ~isnan(pt)
                if obj.ModificationModel.Mode == 0
                    dXY = pt - obj.XYOffset;
                    ptNew = obj.XYInit + dXY;
                    set(obj.FigDoc.XYConnectorLineObj, 'XData', [0 ptNew(1) ptNew(1)], 'YData', [0 0 ptNew(2)]);
                    obj.ModificationModel.Tentative.X = ptNew(1);
                    obj.ModificationModel.Tentative.Y = ptNew(2);
                else
                    theta = atan2(pt(2) - obj.Origin(2), pt(1) - obj.Origin(1));
                    dTheta = theta - obj.ThetaOffset;
                    obj.ModificationModel.Tentative.Theta = obj.ThetaInit + dTheta;
                end

            end
        end

        function windowMouseReleaseCallback(obj, ~, ~)
        %windowMouseReleaseCallback
            if obj.ModificationModel.Mode == 0
                obj.FigDoc.XYConnectorLineObj.Visible = 'off';
            else
                obj.FigDoc.CurrentScanXAxisLineObj.Visible = 'off';
                obj.FigDoc.RefScanXAxisLineObj.Visible = 'off';
            end
            delete(obj.WindowMouseMotionListener);
            delete(obj.WindowMouseReleaseListener);
        end

        function updatePointerCursor(obj, v)
        %updatePointerCursor
            switch(v)
              case 0
                obj.FigDoc.Figure.WindowButtonMotionFcn = @(src, evt) obj.linearMousePointerCallback(src.CurrentPoint);
              case 1
                obj.FigDoc.Figure.WindowButtonMotionFcn = @(src, evt) obj.angularMousePointerCallback(src.CurrentPoint);
              otherwise
                obj.FigDoc.Figure.WindowButtonMotionFcn = [];
                obj.FigDoc.Figure.Pointer = 'arrow';
            end

        end

        function linearMousePointerCallback(obj, pt)
        %linearMousePointerCallback

        % modify mouse pointer only over axes
            axPos = obj.FigDoc.getAxesActualPlotBoxCoordinates;

            if inRange(pt, axPos)
                A = preprocessPointerShapeCData('CursorLinearMod_16x.png');
                obj.FigDoc.Figure.PointerShapeCData = A;
                obj.FigDoc.Figure.Pointer = 'custom';
            else
                obj.FigDoc.Figure.Pointer = 'arrow';
            end
        end

        function angularMousePointerCallback(obj, pt)
        %angularMousePointerCallback

        % modify mouse pointer only over axes
            axPos = obj.FigDoc.getAxesActualPlotBoxCoordinates;
            if inRange(pt, axPos)
                A = preprocessPointerShapeCData('CursorAngularMod_16x.png');
                obj.FigDoc.Figure.PointerShapeCData = A;
                obj.FigDoc.Figure.Pointer = 'custom';
            else
                obj.FigDoc.Figure.Pointer = 'arrow';
            end
        end


        function updateButtonsAndMousePointer(obj, mode)
        %updateButtonsAndMousePointer
            obj.ModificationTab.updateLinearAngularToggleButtons(mode);
            obj.updatePointerCursor(mode);
        end

        function setTabTitle(obj, name)
        %setTabTitle Callback to update modification tab title
            if ~isempty(name) && ischar(name)
                obj.AppViewOrg.ModificationTab.Tab.Title = name;
            end
        end
    end
end


function A = preprocessPointerShapeCData(imgFile)
%preprocessPointerShapeCData
    filePath = fullfile(matlabroot, 'toolbox', 'nav', 'navslamapp', 'icons', imgFile);
    A = double(imread(filePath));
    A(A==0) = nan;
    A(A==2) = 3;
    A(A==1) = 2;
    A(A==3) = 1;
end

function output = inRange(point, boxPosition)
%inRange
    output = false;
    if (point(1) > boxPosition(1)) && (point(1)< boxPosition(1)+boxPosition(3)) && ...
            (point(2) > boxPosition(2)) && (point(2)< boxPosition(2)+boxPosition(4))
        output = true;
    end
end
