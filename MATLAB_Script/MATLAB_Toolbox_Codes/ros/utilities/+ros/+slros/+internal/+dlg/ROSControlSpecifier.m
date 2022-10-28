classdef ROSControlSpecifier < matlab.apps.AppBase
    %This class is for internal use only. It may be removed in the future.

    %  ROSControlSpecifier opens a dialog that lets the user select and
    %  specify the ros_control controller settings. Once the user accepts
    %  the changes, the information is saved in the specified model workspace.
    %
    %  Sample use:
    %   dlg = ros.slros.internal.dlg.ROSControlSpecifier(modelName);

    %   Copyright 2021 The MathWorks, Inc.

    % Properties that correspond to app components
    properties (Access = public)
        FigContainer              matlab.ui.Figure
        GridLayout                matlab.ui.container.GridLayout
        Description               matlab.ui.control.Label
        CancelButton              matlab.ui.control.Button
        OkButton                  matlab.ui.control.Button
        GenCodeCheckbox           matlab.ui.control.CheckBox
        PortTabGroup              matlab.ui.container.TabGroup
        InportTab                 matlab.ui.container.Tab
        OutportTab                matlab.ui.container.Tab
        InportTable               matlab.ui.control.Table
        OutportTable              matlab.ui.control.Table
        ClassNameEditField        matlab.ui.control.EditField
        ClassNameEditFieldLabel   matlab.ui.control.Label
    end

    properties(Constant,Hidden)
        TabCols = struct('PortNum',1,'PortName',2,'ResourceName',3,'ResourceType',4);
        InportTableVarName = 'ROSControlInportTable_';
        OutportTableVarName = 'ROSControlOutportTable_';
        ClassNameVarName = 'ROSControlClassName_';
    end
    
    properties (Access = private)
        ModelName % ModelName
        RootIOPorts % RootIOPorts
        ModelWkspc
    end

    properties(SetAccess=immutable)
        % Resources
        Resources
    end
    
    methods (Access = private)
        function setPositionWrtModelWindow(app)
        % There is no way to set the size of the DDG Window (using MCOS)
        % without specifying the location as well. So set the location
        % relative to the model window
            pos = get_param(app.ModelName, 'Location'); % [x y width height] (in pixels)
                                                        % position the dialog 1/10 of the way from top-left corner
                                                        % When right monitor is primary and model is on left, width
                                                        % can be negative ([-1772 59 -958 589]) - hence using "abs"
            set(0,'units','pixels')  
            pixelSize = get(0,'screensize');
            xyPos = round([pos(1)+ abs(pos(3)/10) pos(2)+ abs(pos(4)/10)]);
            xPos = xyPos(1);
            yPos = pixelSize(4) - (xyPos(2) + app.FigContainer.Position(4));
            wd = app.FigContainer.Position(3);
            ht = app.FigContainer.Position(4);
            app.FigContainer.Position = [xPos yPos wd ht];
        end

        function throwError(app,errTitle,errMsg)
            uialert(app.FigContainer, errMsg, errTitle,'Icon','error','Modal',true);
        end

        function validateInterfaceName(app,event)
            if ~isequal(event.PreviousData,event.NewData) && ...
                    (app.TabCols.ResourceName == event.Indices(2))
                if ~iscvar(event.NewData)
                    throwError(app,message('ros:slros:roscontrol:ResourceNameInvalidTitle').getString,...
                        message('ros:slros:roscontrol:ResourceNameInvalidMsg',event.NewData).getString);
                    event.Source.Data(event.Indices(1),event.Indices(2)) = event.PreviousData;
                end
            end
        end

        function initClassName(app)
            if hasVariable(app.ModelWkspc, app.ClassNameVarName)
                % load data from workspace
                app.ClassNameEditField.Value = getVariable(app.ModelWkspc,app.ClassNameVarName);
            else
                app.ClassNameEditField.Value = 'ControllerHost';
            end
        end

        function validatePortWidthAndDType(app)
            % VALIDATEPORTWIDTHANDDTYPE Validates root I/O port widths and
            % dimensions
            
            % ROS Control package generation is supported for a subset of
            % resources where the width of the port is limited to 1.
            % Also, all the ROS Control resources use double data-type, so
            % the root I/O ports should also have double compiled data
            % types.
            mdlName = app.ModelName;
            portWidths = cell(1,numel(app.RootIOPorts));
            portDTypes = cell(1,numel(app.RootIOPorts));
            cleanupCompile=onCleanup(@()eval(sprintf('%s([],[],[],''term'')', mdlName)));
            eval(sprintf('%s([],[],[],''compile'')', mdlName));
            portFieldMap = containers.Map({'Inport','Outport'},{'Outport','Inport'});
            for k=1:numel(app.RootIOPorts)
                wd = get_param(app.RootIOPorts{k},'CompiledPortWidths');
                dt = get_param(app.RootIOPorts{k},'CompiledPortDataTypes');
                fieldName = portFieldMap(get_param(app.RootIOPorts{k},'BlockType'));
                portWidths{k} = wd.(fieldName);
                portDTypes(k) = dt.(fieldName);
            end
            allPortsDouble = app.RootIOPorts(~contains(portDTypes,'double'));
            allPortsWidth1 = app.RootIOPorts(~cellfun(@(x)isequal(x,1),portWidths));
            if ~isempty(allPortsDouble)
                % Only double data-type is supported
                delete(app.FigContainer);
                error(message('ros:slros:roscontrol:InvalidPortDType', sprintf('''%s''\n',allPortsDouble{:})));
            end
            if ~isempty(allPortsWidth1)
                % Only port width = 1 is supported
                delete(app.FigContainer);
                error(message('ros:slros:roscontrol:InvalidPortWidth', sprintf('''%s''\n',allPortsWidth1{:})));
            end            
        end

        function setPortResource(app, portName, resName, resType)
            set_param([app.ModelName,'/',char(portName)],...
                'UserData', struct('ResourceName',resName,'ResourceType',resType),...
                'UserDataPersistent','on');
        end


        function [resName,resType] = getPortResource(app, portFullName)
            ud = get_param(portFullName,'UserData');
            if isstruct(ud) && isfield(ud,'ResourceName') && isfield(ud,'ResourceType')
                resName = ud.ResourceName;
                resType = ud.ResourceType;
            else                
                resName = lower(matlab.lang.makeValidName(get_param(portFullName,'Name')));
                if isequal(get_param(portFullName,'BlockType'),'Inport')
                    resType = app.Resources.InputResources(1).Name;
                else
                    resType = app.Resources.OutputResources(1).Name;
                end
            end
        end

        function [inportInfoTable,outportInfoTable] = initializePortInfoTable(app)
            allInports = find_system(app.ModelName,'SearchDepth',1,'BlockType','Inport');
            allOutports = find_system(app.ModelName,'SearchDepth',1,'BlockType','Outport');
            app.RootIOPorts = [allInports;allOutports];
            % There must be at least one inport or outport to create a
            % ros_control controller
            mdlName = app.ModelName;
            if isempty(app.RootIOPorts)
                delete(app.FigContainer);
                error(message('ros:slros:roscontrol:NoRootIOPorts',mdlName));
            end
            validatePortWidthAndDType(app);
            numInports = numel(allInports);
            numOutports = numel(allOutports);
            % Validate port dimensions and data-types
            inportInfoTable =  repmat("",numInports,4);
            outportInfoTable =  repmat("",numOutports,4);
            for p = 1:numInports
                thisPort = allInports{p};
                [resName,resType] = getPortResource(app,thisPort);                
                inportInfoTable(p,:) = {...
                    get_param(thisPort,'Port'), ...
                    get_param(thisPort,'PortName'),...
                    resName, resType};
            end
            for p = 1:numOutports
                thisPort = allOutports{p};
                [resName,resType] = getPortResource(app,thisPort);                
                outportInfoTable(p,:) = {...
                    get_param(thisPort,'Port'), ...
                    get_param(thisPort,'PortName'),...
                    resName, resType};
            end
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app, modelName)
            open_system(modelName);            
            app.ModelName = modelName;
            app.Description.Text = getString(message('ros:slros:roscontrol:Description',char(app.ModelName)));
            app.ModelWkspc = get_param(app.ModelName,'ModelWorkspace');
            initClassName(app);
            [app.InportTable.Data,app.OutportTable.Data] = initializePortInfoTable(app);
            setPositionWrtModelWindow(app);
            app.GenCodeCheckbox.Value = ros.codertarget.internal.Util.isROSControlEnabled(modelName);
            % Show the figure after all the data is enumerated
            app.FigContainer.Visible = 'on';
        end

        % Button pushed function: OkButton
        function okButtonPushed(app, ~)
            if ~bdIsLoaded(app.ModelName)
                warning(message('ros:slros:roscontrol:ModelAlreadyClosed',app.ModelName));
                delete(app.FigContainer)
                return;
            end
            w = get_param(app.ModelName,'ModelWorkspace');
            assignin(w,app.InportTableVarName,app.InportTable.Data);
            assignin(w,app.OutportTableVarName,app.OutportTable.Data);
            assignin(w,app.ClassNameVarName,app.ClassNameEditField.Value);
            allResources = [app.InportTable.Data;app.OutportTable.Data];
            for k=1:height(allResources)
                setPortResource(app,allResources(k,2),allResources(k,3),allResources(k,4));
            end
            % Due to a limitation in Simulink Coder licensing, the
            % interface packaging must be set to nonreusable function
            set_param(app.ModelName,'CodeInterfacePackaging','Nonreusable function');
            
            % Apply the setting in the configuration set
            codertarget.data.setParameterValue(...
                getActiveConfigSet(app.ModelName),...
                'ROS.GenerateROSControl',...
                app.GenCodeCheckbox.Value);

            % Close the dialog
            delete(app.FigContainer);
        end

        % Button pushed function: CancelButton
        function cancelButtonPushed(app, ~) 
            % Close the dialog
            delete(app.FigContainer);
            clear app;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create FigContainer and hide until all components are created
            iconPath = fullfile(matlabroot,'toolbox','ros','sltoolstrip','icons','buildAndDeployToRobot_16.png');
            app.FigContainer = uifigure('Visible', 'off', 'Icon',iconPath);
            app.FigContainer.NumberTitle = 'on';
            app.FigContainer.Position = [100 100 600 400];
            app.FigContainer.Name = getString(message('ros:slros:roscontrol:FigTitle'));

            app.GridLayout = uigridlayout(app.FigContainer,...
                [6 4],...
                'RowHeight',{'fit','fit','1x','fit','fit'},... descr,class,port tab,checkbox,buttons
                'ColumnWidth', {'fit','1x',75,75});

            rowNum = 1;

            % Create Description
            app.Description = uilabel(app.GridLayout);
            app.Description.Layout.Row = rowNum;
            app.Description.Layout.Column = [1 4];
            app.Description.Text = getString(message('ros:slros:roscontrol:Description',char(app.ModelName)));

            rowNum = rowNum + 1;

            % Create ClassnameEditFieldLabel
            app.ClassNameEditFieldLabel = uilabel(app.GridLayout);
            app.ClassNameEditFieldLabel.Layout.Row = rowNum;
            app.ClassNameEditFieldLabel.Layout.Column = 1;            
            app.ClassNameEditFieldLabel.Text = getString(message('ros:slros:roscontrol:ClassNameLabel'));

            % Create ClassNameEditField
            app.ClassNameEditField = uieditfield(app.GridLayout, 'text');
            app.ClassNameEditField.Layout.Row = rowNum;
            app.ClassNameEditField.Layout.Column = [2 4];        
            app.ClassNameEditField.Tag = 'ClassnameEdit';
            app.ClassNameEditField.Placeholder = getString(message('ros:slros:roscontrol:ClassNamePlaceHolder'));

            rowNum = rowNum + 1;

            % Create PortTabGroup
            app.PortTabGroup = uitabgroup(app.GridLayout);
            app.PortTabGroup.Layout.Row = rowNum;
            app.PortTabGroup.Layout.Column = [1 4];
            app.InportTab = uitab(app.PortTabGroup,"Title",getString(message('ros:slros:roscontrol:InportTabTitle')));
            app.OutportTab = uitab(app.PortTabGroup,"Title",getString(message('ros:slros:roscontrol:OutportTabTitle')));
            inportTabGridLayout = uigridlayout(app.InportTab,[1 1],'RowHeight',{'fit'},'ColumnWidth',{'fit'});
            outportTabGridLayout= uigridlayout(app.OutportTab,[1 1],'RowHeight',{'fit'},'ColumnWidth',{'fit'});
            
            % Create Inport Table
            app.InportTable = uitable(inportTabGridLayout);
            app.InportTable.Layout.Row = [1 2];
            app.InportTable.Layout.Column = [1 2];
            app.InportTable.ColumnName = {'#'; ...
                getString(message('ros:slros:roscontrol:TableColName'));...
                getString(message('ros:slros:roscontrol:TableColResName'));...
                getString(message('ros:slros:roscontrol:TableColResType'))};
            app.InportTable.RowName = {};
            inputInterfaces = {app.Resources.InputResources.Name};
            app.InportTable.ColumnFormat = ({[] [] [] inputInterfaces});
            app.InportTable.ColumnEditable = [false false true true];
            app.InportTable.CellEditCallback = createCallbackFcn(app, @validateInterfaceName, true);
            app.InportTable.Tooltip = getString(message('ros:slros:roscontrol:TableTooltip'));
            app.InportTable.Tag = 'InportInterfaceMapTable';

            % Create Outport Table
            app.OutportTable = uitable(outportTabGridLayout);
            app.OutportTable.Layout.Row = [1 2];
            app.OutportTable.Layout.Column = [1 2];
            app.OutportTable.ColumnName = {'#';'Name';'Resource name';'Resource type'};
            app.OutportTable.RowName = {};
            outputInterfaces = {app.Resources.OutputResources.Name};
            app.OutportTable.ColumnFormat = ({[] [] [] outputInterfaces});
            app.OutportTable.ColumnEditable = [false false true true];
            app.OutportTable.CellEditCallback = createCallbackFcn(app, @validateInterfaceName, true);
            app.OutportTable.Tooltip = getString(message('ros:slros:roscontrol:TableTooltip'));
            app.OutportTable.Tag = 'OutportInterfaceMapTable';

            rowNum = rowNum + 1;

            app.GenCodeCheckbox = uicheckbox(app.GridLayout);
            app.GenCodeCheckbox.Layout.Row = rowNum;
            app.GenCodeCheckbox.Layout.Column = [1 2];
            app.GenCodeCheckbox.Value = 0;
            app.GenCodeCheckbox.Text = getString(message('ros:slros:roscontrol:GenCodeChkBoxLabel'));
            app.GenCodeCheckbox.Tooltip =  getString(message('ros:slros:roscontrol:GenCodeChkBoxTooltip'));

            rowNum = rowNum + 1;

            % Create OkButton
            app.OkButton = uibutton(app.GridLayout, 'push');
            app.OkButton.ButtonPushedFcn = createCallbackFcn(app, @okButtonPushed, true);
            app.OkButton.Layout.Row = rowNum;
            app.OkButton.Layout.Column = 3;
            app.OkButton.WordWrap = 'on';
            app.OkButton.Text = 'OK';

            % Create CancelButton
            app.CancelButton = uibutton(app.GridLayout, 'push');
            app.CancelButton.ButtonPushedFcn = createCallbackFcn(app, @cancelButtonPushed, true);
            app.CancelButton.Layout.Row = rowNum;
            app.CancelButton.Layout.Column = 4;
            app.CancelButton.Text = 'Cancel';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ROSControlSpecifier(varargin)

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)
                % Instantiate the Input/Output resource names
                app.Resources = ros.codertarget.internal.Util.getROSControlInterfaces();

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.FigContainer)

                % Execute the startup function
                runStartupFcn(app, @(app)startupFcn(app, varargin{:}))
            else

                % Focus the running singleton app
                figure(runningApp.FigContainer)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.FigContainer);
        end
    end

end
