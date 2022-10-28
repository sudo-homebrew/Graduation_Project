classdef SolverInfoView < robotics.ikdesigner.internal.view.View
%This class is for internal use only and may be removed in a future release

%SolverInfoView View for displaying latest solver data

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)

        %DESCRIPTIONTEXT Text displayed above the loading window
        DESCRIPTIONTEXT = ...
            string(message('robotics:ikdesigner:solverinfoview:DescriptionText'));

        %WINDOWNAME Name displayed on the uifigure
        WINDOWNAME = ...
            string(message('robotics:ikdesigner:solverinfoview:WindowName'));

        %EXIT_FLAG_LABEL Label text for "ExitFlag" in solution info
        EXIT_FLAG_LABEL = ...
            string(message('robotics:ikdesigner:solverinfoview:ExitFlagLabel'));

        %EXIT_FLAG_HYPERLINKCLICKED_FN MathWorks Documentation URL on Exit Flag
        EXIT_FLAG_HYPERLINKCLICKED_FN = ...
            @(~,~)helpview(fullfile(docroot,'toolbox','robotics','helptargets.map'),'solverExitFlags');

        %STATUS_LABEL Label text for "Status" in solution info
        STATUS_LABEL = ...
            string(message('robotics:ikdesigner:solverinfoview:StatusLabel'));

        %NUM_RANDOM_RESTARTS_LABEL Label text for "NumRandomRestarts" in solution info
        NUM_RANDOM_RESTARTS_LABEL = ...
            string(message('robotics:ikdesigner:solverinfoview:NumRandomRestartsLabel'));

        %ITERATIONS_LABEL Label text for "Iterations" in solution info
        ITERATIONS_LABEL = ...
            string(message('robotics:ikdesigner:solverinfoview:IterationsLabel'));

        %CONSTRAINT_VIOLATIONS_LABEL Label text for "ConstraintViolations" in solution info
        CONSTRAINT_VIOLATIONS_LABEL = ...
            string(message('robotics:ikdesigner:solverinfoview:ConstraintViolationsLabel'));

        %CONSTRAINT_VIOLATIONS_HYPERLINK_FORMAT_TEXT Hyperlink text for "ConstraintViolations" in solution info
        CONSTRAINT_VIOLATIONS_HYPERLINK_FORMAT_TEXT = ...
            string(message('robotics:ikdesigner:solverinfoview:ConstraintViolationsFormatText'));

        %NOT_RUN Status text indicating the solver hasn't been run
        NOT_RUN = string(message('robotics:ikdesigner:solverinfoview:SolverNotRun'));

        %SEE_COMMAND_WINDOW_FOR_MORE_DETAILS
        SEE_COMMAND_WINDOW_FOR_MORE_DETAILS= ...
            string(message('robotics:ikdesigner:solverinfoview:CommandWindowForDetails'));

    end

    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.SolverInfoViewTester})

        %Window Dialog uifigure
        Window

        %Parent Parent object to who the event requests are forwarded
        Parent

        %Iterations Field which displays iterations in solution info
        IterationsField

        %NumRandomRestartsField Displays Number of Random Restarts in solution info
        NumRandomRestartsField

        %ConstraintViolationsField Displays the violations of every constraint in solution info
        ConstraintViolationsField

        %ExitFlagField Displays exit flag of the solution info
        ExitFlagField

        %StatusField Displays the status of the solution info
        StatusField

        %IsWindowOpen Property to check whether window is open
        %   This boolean ensures that at most one modal UI window can be
        %   open at a time.
        IsWindowOpen
    end

    methods
        function obj = SolverInfoView(parentComponent)
        %CustomDataUI Constructor

        % Initialize core properties
            obj.Parent = parentComponent;

            obj.initialize();
        end

        function initialize(obj)
        %initialize Initialize the view

            obj.IsWindowOpen = false;

        end

        function sendNotification(obj, eventName, eventData)
        %sendNotification Send notification
        %   This method tells the parent to send the notification. This
        %   ensures that the notifications are only ever sent by the
        %   main toolstrip object.

            obj.Parent.sendNotification(eventName, eventData);
        end

        function show(obj)
        %show Show the figure in front of the app
        
            if obj.IsWindowOpen
                % TODO: Bring the figure to front
                return;
            else
                obj.IsWindowOpen = true;
            end

            figGrid = obj.createBaselineUI();
            obj.buildDescriptionArea(figGrid);
            obj.sendNotification('RequestLastSolutionInfo',[]);

        end

        function hide(obj)
        %hide Hide the figure
        %   Close the figure and reset the app state

            if ~isempty(obj.Window) && isvalid(obj.Window)
                delete(obj.Window);
            end

            obj.IsWindowOpen = false;

            obj.Window = [];
            obj.IterationsField = [];
            obj.NumRandomRestartsField = [];
            obj.ConstraintViolationsField=[];
            obj.ExitFlagField=[];
            obj.StatusField=[];

        end

        function updateFromSolutionInfo(obj,info)
        %updateFromSolutionInfo Assign the info values to fields
            if(~isempty(info))
                obj.updateWindowTitleWithDatetimeNow();
                obj.IterationsField.Text=num2str(info.Iterations);
                obj.ConstraintViolationsField.Text=...
                    sprintf(obj.CONSTRAINT_VIOLATIONS_HYPERLINK_FORMAT_TEXT,size(info.ConstraintViolations));
                obj.ConstraintViolationsField.HyperlinkClickedFcn=...
                    @(~,~)(dispConstraintsViolationsStruct(info.ConstraintViolations));
                obj.StatusField.Text=info.Status;
                obj.ExitFlagField.Text=num2str(info.ExitFlag);
                obj.NumRandomRestartsField.Text=num2str(info.NumRandomRestarts);
            end
        end
    end

    methods (Access = private)

        function updateWindowTitleWithDatetimeNow(obj)
            %updateWindowTitleWithDatetimeNow Update window time stamp
            %   If the window is open, update its time stamp

            if obj.IsWindowOpen
                currenttime=datetime('now');
                obj.Window.Name=sprintf("%s %s",obj.WINDOWNAME,string(currenttime));
            end
        end

        function setFieldTextToNotRun(obj,field)
            field.Text=obj.NOT_RUN;
        end

        function figGrid = createBaselineUI(obj)
        %createBaselineUI Create a UIFigure window

            obj.Window = uifigure('Name', obj.WINDOWNAME, 'Scrollable',true);
            obj.updateWindowTitleWithDatetimeNow();
            obj.Window.Resize = matlab.lang.OnOffSwitchState.on;

            % Create a grid within which to place the objects in the figure
            figGrid = uigridlayout(obj.Window);

            % Update the close request function, which is the X in the
            % upper right corner
            obj.Window.CloseRequestFcn = @(src,evt)hide(obj);

            % Define the figure height
            obj.Window.Position(4) = 200;
        end

        function buildDescriptionArea(obj, figGrid)
        %buildDescriptionArea Add a description to the uifigure
        %   Add a uilabel containing a description of the uifigure to
        %   instruct the user how to use the UI.

        %Add a text area in the first row of the grid
            descriptionText = uilabel(figGrid);
            descriptionText.Layout.Column = [1 2];
            descriptionText.Text = obj.DESCRIPTIONTEXT;
            descriptionText.FontWeight='bold';
            obj.makeIterationsArea(figGrid);
            obj.makeNumRandomRestartsArea(figGrid);
            obj.makeConstraintViolationsArea(figGrid);
            obj.makeStatusArea(figGrid);
            obj.makeExitFlagArea(figGrid);
            figGrid.RowHeight=repmat({'fit'},1,numel(figGrid.RowHeight));
        end

    end
    methods(Access=private)
        function formatLabelAndField(~,labelelem)
            labelelem.HorizontalAlignment='left';
        end


        function makeExitFlagArea(obj,figGrid)
            hyperlink = uihyperlink(figGrid);
            hyperlink.Text=obj.EXIT_FLAG_LABEL;
            hyperlink.HyperlinkClickedFcn=obj.EXIT_FLAG_HYPERLINKCLICKED_FN;
            obj.ExitFlagField=uilabel(figGrid);
            obj.formatLabelAndField(hyperlink);
            obj.setFieldTextToNotRun(obj.ExitFlagField);
        end

        function makeIterationsArea(obj,figGrid)
            label = uilabel(figGrid);
            label.Text=obj.ITERATIONS_LABEL;
            obj.IterationsField=uilabel(figGrid);
            obj.formatLabelAndField(label);
            obj.setFieldTextToNotRun(obj.IterationsField);
        end

        function makeNumRandomRestartsArea(obj,figGrid)
            label = uilabel(figGrid);
            label.Text=obj.NUM_RANDOM_RESTARTS_LABEL;
            obj.NumRandomRestartsField=uilabel(figGrid);
            obj.formatLabelAndField(label);
            obj.setFieldTextToNotRun(obj.NumRandomRestartsField);
        end

        function makeStatusArea(obj,figGrid)
            label = uilabel(figGrid);
            label.Text=obj.STATUS_LABEL;
            obj.StatusField=uilabel(figGrid);
            obj.formatLabelAndField(label);
            obj.setFieldTextToNotRun(obj.StatusField);
        end

        function makeConstraintViolationsArea(obj,figGrid)
            label = uilabel(figGrid);
            label.Text=obj.CONSTRAINT_VIOLATIONS_LABEL;
            obj.ConstraintViolationsField=uihyperlink(figGrid);
            obj.ConstraintViolationsField.Tooltip=obj.SEE_COMMAND_WINDOW_FOR_MORE_DETAILS;
            obj.formatLabelAndField(label);
            obj.setFieldTextToNotRun(obj.ConstraintViolationsField);
        end

    end
end
function dispConstraintsViolationsStruct(s)
    disp('***')
    arrayfun(@disp,s);
    disp('***')
end
