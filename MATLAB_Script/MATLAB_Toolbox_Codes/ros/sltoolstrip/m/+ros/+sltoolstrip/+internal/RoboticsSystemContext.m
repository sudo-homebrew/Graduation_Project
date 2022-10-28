classdef RoboticsSystemContext < coder.internal.toolstrip.HardwareBoardContext
    %RoboticsSystemContext - Robot Operating System (ROS) Toolstrip App Context
    %
    
    %   Copyright 2018-2021 The MathWorks, Inc.
    
    properties( Constant )
        BuildActionNone = 'None';
        BuildActionBuild = 'Build';
        BuildActionBuildAndLoad = 'Build and load';
        BuildActionBuildAndRun  = 'Build and run';
        BuildActionNoneFmt = 'none';
        BuildActionBuildFmt = 'build';
        BuildActionBuildAndLoadFmt = 'buildandload';
        BuildActionBuildAndRunFmt = 'buildandrun';
    end
    
    properties( SetAccess=private )
        AppTypeChain
        HardwareBoardHandler
        
        
        BuildEnabled = true;
        BuildAndDeployEnabled = true;
        BuildDeployAndStartEnabled  = true;
        
        BuildAction = [];
        HardwareBoard = [];
        RobotToBaseBuildActionMap = containers.Map( ...
            {ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionNoneFmt,...
            ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildFmt,...
            ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildAndLoadFmt, ...
            ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildAndRunFmt},...
            {'Build',...
            'Build',...
            'BuildAndDeploy',...
            'BuildDeployAndStart'}); %#ok<MCHDP>
        SupportedBoards
        DeviceNames
        RemoteBuildFlag
        RaspiROSSelected = false;
        IsROSControlEnabled = false;
    end
    
    methods
        function obj = RoboticsSystemContext( app, model )
            obj = obj@coder.internal.toolstrip.HardwareBoardContext( app, model );
            obj.HardwareBoardHandler = ros.sltoolstrip.internal.RoboticsHardwareBoardHandler( 'ROS', get_param( model, 'Name' ) );
            obj.AppTypeChain = { 'roboticsSystemToolboxContext' };
            obj.HardwareBoard = get_param(model, 'HardwareBoard');
            obj.setBuildActionsForHardwareBoard(model);
            obj.BuildAction = obj.getModelBuildAction(model);
            cset = getActiveConfigSet(model);
            obj.IsROSControlEnabled = ros.codertarget.internal.Util.isROSControlEnabled(model);
            obj.RemoteBuildFlag = ros.codertarget.internal.isRemoteBuild(cset);
            obj.setDeployContext( obj.getDeployContext() );
            obj.setExtModeContext( 'HWBoardDeployedContext' );
            obj.SupportedBoards = ros.sltoolstrip.internal.getSupportedHardwareBoards;
            obj.DeviceNames = {message('ros:slros:toolstrip:Localhost').getString, ...
                message('ros:slros:toolstrip:RemoteDevice').getString};
            obj.setDeviceAddressForOlderModels(model);            
            obj.RobotToBaseBuildActionMap = containers.Map( ...
                {ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionNoneFmt,...
                ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildFmt,...
                ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildAndLoadFmt, ...
                ros.sltoolstrip.internal.RoboticsSystemContext.BuildActionBuildAndRunFmt},...
                {'Build',...
                'Build',...
                'BuildAndDeploy',...
                'BuildDeployAndStart'});
        end
        
        function typeChain = getAppTypeChain( obj )
            obj.AppTypeChain = {'roboticsSystemToolboxContext'};
            typeChain = [obj.getROSControlContext(),obj.AppTypeChain];
        end
        
        function removeConfiguration( ~, cs )
            cs.set_param( 'SystemTargetFile', 'grt.tlc' );
            cs.set_param( 'HardwareBoard', 'None' );
        end
    end
    
    methods( Access = protected )
        function handler = getHardwareBoardHandler( obj )
            handler = obj.HardwareBoardHandler;
        end

        function context = getROSControlDeployContext(this, ~)
            context = this.DeployContext;
            if strcmp(this.BuildAction, 'Build') % user selected build
                if ~this.RemoteBuildFlag
                    context = 'BuildSelectedLocalDeployROSControlContext';
                elseif this.BuildAndDeployEnabled
                    context = 'BuildSelectedBuildDeployEnabledROSControlContext';
                else
                    context = 'BuildSelectedROSControlContext';
                end
            elseif strcmp(this.BuildAction,'BuildAndDeploy') % user selected build and load
                if this.BuildEnabled
                    context = 'BuildDeploySelectedBuildEnabledROSControlContext';
                else
                    context = 'BuildDeploySelectedRobotContext';
                end
            end
        end

        function context = getDeployContext(this, ~)
            context = this.DeployContext;
            if this.IsROSControlEnabled
                context = this.getROSControlDeployContext();
            else
                if strcmp(this.BuildAction, 'Build') % user selected build
                    if ~this.RemoteBuildFlag
                        context = 'BuildSelectedLocalDeploy';
                    elseif this.BuildAndDeployEnabled && ~this.BuildDeployAndStartEnabled
                        context = 'BuildSelectedBuildDeployEnabledRobotContext';
                    elseif ~this.BuildAndDeployEnabled && this.BuildDeployAndStartEnabled
                        context = 'BuildSelectedBuildDeployStartEnabledRobotContext';
                    elseif this.BuildAndDeployEnabled && this.BuildDeployAndStartEnabled
                        context = 'BuildSelectedAllBuildsEnabledRobotContext';
                    else
                        context = 'BuildSelectedRobotContext';
                    end
                elseif strcmp(this.BuildAction, 'BuildAndDeploy') % user selected build and load
                    if this.BuildEnabled && ~this.BuildDeployAndStartEnabled
                        context = 'BuildDeploySelectedBuildEnabledRobotContext';
                    elseif ~this.BuildEnabled && this.BuildDeployAndStartEnabled
                        context = 'BuildDeploySelectedBuildDeployStartEnabledRobotContext';
                    elseif this.BuildEnabled && this.BuildDeployAndStartEnabled
                        context = 'BuildDeploySelectedAllBuildsEnabledRobotContext';
                    else
                        context = 'BuildDeploySelectedRobotContext';
                    end
                elseif strcmp(this.BuildAction, 'BuildDeployAndStart') % user selected build, load and run
                    if ~this.RemoteBuildFlag
                        context = 'BuildStartSelectedLocalDeploy';
                    elseif this.BuildEnabled && ~this.BuildAndDeployEnabled
                        context = 'BuildDeployStartSelectedBuildEnabledRobotContext';
                    elseif ~this.BuildEnabled && this.BuildAndDeployEnabled
                        context = 'BuildDeployStartSelectedBuildDeployEnabledRobotContext';
                    elseif this.BuildEnabled && this.BuildAndDeployEnabled
                        context = 'BuildDeployStartSelectedAllBuildsEnabledRobotContext';
                    else
                        context = 'BuildDeployStartSelectedRobotContext';
                    end
                end
            end
        end
        
        function buildHardwareBoardActionCBInternal( this, cbinfo )
            this.standaloneBuild(cbinfo, this.BuildActionNone);
        end
        
        function buildDeployHardwareBoardActionCBInternal( this, cbinfo )
            this.standaloneBuild(cbinfo, this.BuildActionBuildAndLoad);
        end
        
        function buildDeployStartHardwareBoardActionCBInternal( this, cbinfo )
            this.standaloneBuild(cbinfo, this.BuildActionBuildAndRun);
        end
        
        function context = getROSControlContext(this)
            if isequal(message('ros:slros:cgen:ui_hwboard').getString,this.HardwareBoard)
                context = 'ROSControlSettingsVisibleContext';
            else
                context = 'ROSControlSettingsNotVisibleContext';
            end
        end
        
    end
    
    methods( Access = private )
        function standaloneBuild(this, cbinfo, selectedBuildType)
            model = SLStudio.Utils.getModelName(cbinfo);
            cs = getActiveConfigSet(model);
            buildType = codertarget.data.getParameterValue(cs, 'Runtime.BuildAction');
            if ~isempty(selectedBuildType) && ~isequal(buildType, selectedBuildType) 
                codertarget.data.setParameterValue(cs, 'Runtime.BuildAction', selectedBuildType);
                this.BuildAction = this.formatBuildAction(selectedBuildType);
                % since refreshers are run asynchronously, update the
                % context to immediately reflect the change in the
                % toolstrip
                this.setDeployContext(this.getDeployContext());
                cs.refreshDialog();
            end
            this.HardwareBoardHandler.standaloneBuild(cbinfo);
        end
        
        function ret = formatBuildAction(~, entries)
            ret =  regexprep(lower(entries), '[\s,]', '');
        end
        
        function val = getModelBuildAction(this, model)
            %This maps {'None', 'Build and load', 'Build and run'} ->
            %{'Build','BuildAndDeploy','BuildDeployAndStart'}
            %This translation is needed to reuse HardwareBoardContext
            cs = getActiveConfigSet(model);
            val = '';
            if codertarget.data.isParameterInitialized(cs, 'Runtime.BuildAction')
                buildAction = codertarget.data.getParameterValue(cs, 'Runtime.BuildAction');
                buildAction = this.formatBuildAction(buildAction);
                val = this.RobotToBaseBuildActionMap(buildAction);
            end
        end
        
        function setBuildActionsForHardwareBoard(this, model)
            entries = codertarget.parameter.getBuildOptionsEntries(model);
            if ~isempty(entries)
                entries = this.formatBuildAction(entries);
                this.BuildEnabled = any(startsWith(entries,this.BuildActionNoneFmt)) || any(strcmp(entries,this.BuildActionBuild));
                this.BuildAndDeployEnabled = any(startsWith(entries,this.BuildActionBuildAndLoadFmt));
                this.BuildDeployAndStartEnabled = any(startsWith(entries,this.BuildActionBuildAndRunFmt));
            end
        end
        
        function setDeviceAddressForOlderModels(this, model)
            % SETDEVICEADDRESSFOROLDERMODELS Set the device address
            % parameter for use with parameter tuning to the correct value
            if ismember(this.HardwareBoard,this.SupportedBoards) && ~contains(this.HardwareBoard,'Raspberry')
                cset = getActiveConfigSet(model);
                ctdata = codertarget.data.getData(cset);
                ipAddrCallback = 'ros.codertarget.internal.getExtmodeDeviceAddress(hCS)';
                if ~strcmp(ctdata.ConnectionInfo.TCPIP.IPAddress,ipAddrCallback)
                    prevVal = get_param(model,'Dirty');
                    ctdata.ConnectionInfo.TCPIP.IPAddress = ipAddrCallback;
                    codertarget.data.setData(cset,ctdata);
                    set_param(model,'Dirty',prevVal);
                end
            end
        end          
    end
    
    methods( Static )
        
        % Each HardwareBoardContext must provide this method and return
        % true if the corresponding app is ready to adopt the "dialog-based"
        % approach described in g1926763 and 1926765
        % TODO: remove this method (and update the HardwareBoardContextManager)
        % once the migration of all the apps is completed
        function isSupported = isDialogSupported()
            isSupported = true;
        end
        
        function dlg = setupConfigSet(model, cs )
            dlg = ros.sltoolstrip.internal.SetupConfigSetDialog(model, cs);
        end
        
        function configureRosNetworkCB(~)
            %configureRosNetworkCB Open the dialog for setting the ROS network
            ros.slros.internal.dlg.NetworkAddressSpecifier.launch();
        end
        
        function openVarSizeDialogCB(~)
            %openVarSizeDialogCB Open the dialog for adjusting the variable-size array settings
            ros.slros.internal.dlg.ArraySizeManager.launch(bdroot(gcs));
        end
        
        function openROSControlApp(~)
            %openROSControlApp Open the ROS Control App to specify the port
            %interface settings and build ros_control controller ROS
            %package
            ros.slros.internal.dlg.ROSControlSpecifier(bdroot(gcs));
        end

        function openConnectToRobotCB(~)
            %openConnectToRobotCB Open the dialog for ROS target connection settings
            ros.slros.internal.dlg.DeviceParameterSpecifier.openDialogForModel(...
                bdroot(gcs));
        end
        
        function testRobotConnectionCB(~)
            %testRobotConnectionCB Test the ROS target connection settings
            modelName = bdroot(gcs);
            deviceParams = ros.codertarget.internal.DeviceParameters;
            [hostName, sshPort, userName, password, ~, catkinWs, rosInstall, ...
                ros2Ws, ros2Install] = deviceParams.getDeviceParameters;
            if isequal(get_param(modelName,'HardwareBoard'),...
                    message('ros:slros2:codegen:ui_hwboard').getString)
                rosVer = 'ROS2';
                wsFolder = ros2Ws;
                installFolder = ros2Install;
            else
                rosVer = 'ROS';
                wsFolder = catkinWs;
                installFolder = rosInstall;
            end
            tester = ros.slros.internal.diag.DeviceDiagnostics(modelName,rosVer);
            tester.runDiagnostics(hostName, sshPort, userName, ...
                password, installFolder, wsFolder);
        end
        
        function updateDeviceTypeSettingsCB(cbinfo)
            cset = getActiveConfigSet(cbinfo.model.Handle);
            ctData = codertarget.data.getData(cset);
            context = coder.internal.toolstrip.HardwareBoardContextManager.getContext(cbinfo.model.Handle);
            if ~isempty(ctData)
                if strcmpi(cbinfo.EventData, message('ros:slros:toolstrip:Localhost').getString)
                    ctData.ROS.RemoteBuild = false;
                    codertarget.data.setData(cset, ctData);
                    context.RemoteBuildFlag = false;
                else
                    ctData.ROS.RemoteBuild = true;
                    codertarget.data.setData(cset, ctData);
                    context.RemoteBuildFlag = true;
                end
            end
        end

        function updateDeviceTypeSettingsRF(cbinfo, action)
            context = coder.internal.toolstrip.HardwareBoardContextManager.getContext(cbinfo.model.Handle);
            rosContextName = 'ros.sltoolstrip.internal.RoboticsSystemContext';
            if ~isempty(context) && isa(context,rosContextName)
                cset = getActiveConfigSet(cbinfo.model.Handle);
                context.RemoteBuildFlag = ros.codertarget.internal.isRemoteBuild(cset);
                remoteDevString = message('ros:slros:toolstrip:RemoteDevice').getString;
                dev = ros.codertarget.internal.DeviceParameters;
                if isempty(dev.getHostname)
                    newEntries = context.DeviceNames;
                else
                    newEntries = strrep(context.DeviceNames, ...
                        remoteDevString,[remoteDevString,' (',dev.getHostname,')']);
                end
                if context.RaspiROSSelected
                    % remove "Localhost" selection for raspberry pi
                    action.validateAndSetEntries(newEntries(end));
                    action.selectedItem = newEntries{end};
                else
                    action.validateAndSetEntries(newEntries);
                    action.selectedItem = newEntries{[~context.RemoteBuildFlag context.RemoteBuildFlag]};
                end
                if ~isequal(get_param(cbinfo.model.Handle, 'ExtModeConnected'), 'on')
                    action.enabled = context.IsConfigSetWritable;
                    context.setDeployContext(context.getDeployContext());
                end
            end
        end
        
        function updateHardwareBoardSettingsCB(cbinfo)
            set_param(cbinfo.model.Handle, 'HardwareBoard', cbinfo.EventData);
        end
        
        function updateHardwareBoardSettingsRF(cbinfo, action)
            selectedBoard = get_param(cbinfo.model.Handle, 'HardwareBoard');
            context = coder.internal.toolstrip.HardwareBoardContextManager.getContext(cbinfo.model.Handle);
            if ~isempty( context ) && isa( context, 'ros.sltoolstrip.internal.RoboticsSystemContext' )
                selectedBuildAction = context.getModelBuildAction(cbinfo.model.Handle);
                if isempty( selectedBuildAction ) || isempty( selectedBoard )
                    % model is not configured for ROS
                    % no action needed
                    return;
                end
                if ismember(selectedBoard, context.SupportedBoards)
                    action.validateAndSetEntries(context.SupportedBoards);
                else
                    %When we are unable to find the selected board in the
                    %list of supported boards, just display that board.
                    action.validateAndSetEntries({selectedBoard});
                end
                rosControlEnabled = ros.codertarget.internal.Util.isROSControlEnabled(cbinfo.model.Handle);
                action.selectedItem = selectedBoard;
                if ~isequal(get_param(cbinfo.model.Handle, 'ExtModeConnected'), 'on') % model is connected for external mode
                    action.enabled = context.IsConfigSetWritable;
                    if ~strcmp(context.BuildAction, selectedBuildAction) ||...
                            ~strcmp(context.HardwareBoard, selectedBoard) || ...
                            ~isequal(context.IsROSControlEnabled,rosControlEnabled)
                        % Runtime.BuildAction has changed
                        context.HardwareBoard = selectedBoard;
                        context.BuildAction = selectedBuildAction;
                        context.IsROSControlEnabled = rosControlEnabled;
                        context.setBuildActionsForHardwareBoard(cbinfo.model.Handle);
                        context.setDeployContext(context.getDeployContext());
                    end
                end
            end
        end
    end
end
