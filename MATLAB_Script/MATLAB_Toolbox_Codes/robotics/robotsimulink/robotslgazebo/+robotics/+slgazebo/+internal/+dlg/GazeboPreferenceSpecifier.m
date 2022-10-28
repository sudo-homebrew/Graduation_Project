classdef GazeboPreferenceSpecifier < handle
%This class is for internal use only. It may be removed in the future.

%  GazeboPreferenceSpecifier opens a DDG dialog that lets the user load and
%  save Gazebo preference information to MATLAB Preferences.
%
%  Sample use:
%   addr = PreferenceSpecifier;
%   addr.openDialog

%   Copyright 2019-2020 The MathWorks, Inc.

    properties(SetAccess=private)
        PrefStore
        Profile = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile.empty;
    end

    properties(Constant)
        GazeboPreferenceHelpTag = 'helptext';
        GazeboHostSelectTag = 'masterhost';
        GazeboHostNameTag = 'masterHostName';
        GazeboHostPortTag = 'masterPortNum';
        GazeboTestConnectTag = 'masterTestBtn';
        SimulationTimoutTag = 'simulationTimeout';
    end

    methods

        function obj = GazeboPreferenceSpecifier()
            obj.PrefStore = robotics.slgazebo.internal.sim.GazeboPreferenceStore;
            obj.Profile = obj.PrefStore.getProfile();
        end

        function dlg = openDialog(obj)
            dlg = DAStudio.Dialog(obj);
        end

        function dlgClose(obj, closeaction)
        % close action is
        %   'ok' if user clicked OK
        %   'cancel' if user clicked cancel or closed window
            try
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                if isAcceptedSelection
                    obj.PrefStore.setProfile(obj.Profile);
                    obj.PrefStore.updateStore();
                end
            catch ME
                disp(ME.getReport);
                % Absorb all errors. If they are propagated back to
                % DDG, this causes MATLAB to crash, (Can't convert to
                % warnings are not as they are not displayed either).
            end
        end


        function hostSelectionChanged(obj, dlg, value)
        % value: 0 (Default), 1 (Custom)
            obj.Profile.MasterUseDefault = (value == 0);
            dlg.refresh;
        end

        function hostNameChanged(obj, dlg, tag, value)
            
            oldStr = obj.Profile.MasterHost;
            
            % if user input empty host name, directly revert to old value
            % and return
            if isempty(value)
                dlg.setWidgetValue(tag, oldStr);
                return
            end

            % process the value to make it a legal host name
            newStr = robotics.slcore.internal.dlg.PreferenceSpecifierUtil.convertStrToHostName(value);
            if isempty(newStr)
                dlg.setWidgetValue(tag, oldStr);
                error(message('robotics:robotslgazebo:preferencedlg:InvalidHost').getString);
            end

            obj.Profile.MasterHost = newStr;

            dlg.setWidgetValue(tag, newStr);
            
            % show error if we have adjusted the hostname user entered
            if ~strcmp(newStr, value)
                error(message('robotics:robotslgazebo:preferencedlg:HostNameAdjusted', value, newStr).getString);
            end
        end

        function portChanged(obj, dlg, tag, value)
            
            % if user input empty host port, directly revert to old value
            % and return
            if isempty(value)
                dlg.setWidgetValue(tag, sprintf('%d', obj.Profile.MasterPort));
                return
            end
            
            [num, numIsValid] = robotics.slcore.internal.dlg.PreferenceSpecifierUtil.convertStrToPortNum(value);
            if numIsValid
                obj.Profile.MasterPort = num;
            end
            valueStr = sprintf('%d', obj.Profile.MasterPort);
            dlg.setWidgetValue(tag, valueStr);
            
            if ~numIsValid
                error(message('robotics:robotslgazebo:preferencedlg:PortMustBeNumeric').getString);
            end
            
            if num ~= str2double(value)
                error(message('robotics:robotslgazebo:preferencedlg:NumericPortAdjusted', value, num).getString);
            end
        end

        function simTimeoutChanged(obj, dlg, tag, value)
            % if user input empty step time, directly revert to old value
            % and return
            if isempty(value)
                dlg.setWidgetValue(tag, num2str(obj.Profile.SimulationTimeout));
                return
            end
            
            num = str2double(value);
            try
                obj.Profile.SimulationTimeout = num;
            catch EX
                valueStr = num2str(obj.Profile.SimulationTimeout);
                dlg.setWidgetValue(tag, valueStr);
                throw(EX);
            end
            valueStr = num2str(obj.Profile.SimulationTimeout);
            dlg.setWidgetValue(tag, valueStr);
        end

        function testMasterConnection(obj, dlg, tag) %#ok<INUSD>
            dlgTitle = message('robotics:robotslgazebo:preferencedlg:TestMasterDlgTitle').getString;

            dp = DAStudio.DialogProvider;

            % Gazebo client connect can take a while since it attempts
            % to resolve hostname  (and throws error if it cannot). So,
            % put up the waiting dialog before invoking ROSMaster
            % Note -- MSGBOX ignores dp.DialogImage

            waitingDlg = dp.msgbox(...
                message('robotics:robotslgazebo:preferencedlg:TestMasterChecking').getString, ...
                dlgTitle, true);
            client = robotics.internal.GazeboClient;
            try
                client.connect(obj.Profile.MasterHost, obj.Profile.MasterPort, 500);
                delete(waitingDlg);
                dp.msgbox(...
                    message('robotics:robotslgazebo:preferencedlg:TestMasterSuccess', ...
                            obj.Profile.MasterHost).getString, ...
                    dlgTitle, true);

            catch ME
                if isa(waitingDlg, 'DAStudio.Dialog') && ishandle(waitingDlg)
                    delete(waitingDlg);
                end
                throw(ME);
            end

        end


        function dlgstruct = getDialogSchema(obj)
        %% "Gazebo Plugin Server" section

            row=1;

            masterHostLabel.Name = message('robotics:robotslgazebo:preferencedlg:NetworkAddress').getString;
            masterHostLabel.Type  = 'text';
            masterHostLabel.Alignment  = 7;
            masterHostLabel.RowSpan = [row row];
            masterHostLabel.ColSpan = [1 1];
            masterHostLabel.Visible = true;

            masterHostSelection.Name = '';
            masterHostSelection.NameLocation = 6;
            masterHostSelection.Type  = 'combobox';
            masterHostSelection.Alignment = 0;
            masterHostSelection.RowSpan = [row row];
            masterHostSelection.ColSpan = [2 2];
            masterHostSelection.Entries = {
                message('robotics:robotslgazebo:preferencedlg:NetAddrDefault').getString
                message('robotics:robotslgazebo:preferencedlg:NetAddrCustom').getString
                   };

            if obj.Profile.MasterUseDefault
                masterHostSelection.Value = 0;
            else
                masterHostSelection.Value = 1;
            end

            masterHostSelection.ObjectMethod = 'hostSelectionChanged'; % call method on UDD source object
            masterHostSelection.MethodArgs = {'%dialog', '%value'}; % '%handle ' is implicit as first arg
            masterHostSelection.ArgDataTypes = {'handle', 'mxArray'};
            masterHostSelection.Tag = obj.GazeboHostSelectTag;

            masterTestBtn.Name = message('robotics:robotslgazebo:preferencedlg:TestConnection').getString;
            masterTestBtn.Type = 'pushbutton';
            masterTestBtn.RowSpan = [row row];
            masterTestBtn.ColSpan = [3 3];
            masterTestBtn.Alignment = 6; % top-left
            masterTestBtn.Tag = obj.GazeboTestConnectTag;
            masterTestBtn.ObjectMethod = 'testMasterConnection'; % call method on UDD source object
            masterTestBtn.MethodArgs = {'%dialog', '%tag'}; % '%handle ' is implicit as first arg
            masterTestBtn.ArgDataTypes = {'handle', 'string'};

            row = row+1;

            masterHostNameLabel.Name = message('robotics:robotslgazebo:preferencedlg:HostOrIPAddr').getString;
            masterHostNameLabel.Type  = 'text';
            masterHostNameLabel.Alignment  = 7;
            masterHostNameLabel.RowSpan = [row row];
            masterHostNameLabel.ColSpan = [1 1];
            masterHostNameLabel.Visible = ~obj.Profile.MasterUseDefault;

            masterHostName.Name = '';
            masterHostName.Type  = 'edit';
            masterHostName.Value = obj.Profile.MasterHost;
            masterHostName.RowSpan = [row row];
            masterHostName.ColSpan = [2 2];
            masterHostName.Alignment = 0; % top-left
            masterHostName.Tag = obj.GazeboHostNameTag;
            masterHostName.ObjectMethod = 'hostNameChanged'; % call method on UDD source object
            masterHostName.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            masterHostName.ArgDataTypes = {'handle', 'string', 'mxArray'};
            masterHostName.Visible = ~obj.Profile.MasterUseDefault;

            row = row+1;

            masterPortNumLabel.Name = message('robotics:robotslgazebo:preferencedlg:Port').getString;
            masterPortNumLabel.Type  = 'text';
            masterPortNumLabel.Alignment  = 7;
            masterPortNumLabel.RowSpan = [row row];
            masterPortNumLabel.ColSpan = [1 1];
            masterPortNumLabel.Visible = ~obj.Profile.MasterUseDefault;

            masterPortNum.Name = '';
            masterPortNum.Type = 'edit';
            masterPortNum.Value = sprintf('%d', obj.Profile.MasterPort);
            masterPortNum.RowSpan = [row row];
            masterPortNum.ColSpan = [2 2];
            masterPortNum.Alignment = 0; % top-left
            masterPortNum.Tag = obj.GazeboHostPortTag;
            masterPortNum.ObjectMethod = 'portChanged'; % call method on UDD source object
            masterPortNum.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            masterPortNum.ArgDataTypes = {'handle', 'string', 'mxArray'};
            masterPortNum.Visible = ~obj.Profile.MasterUseDefault;

            row = row + 1;

            masterDefaultHelp.Name = message('robotics:robotslgazebo:preferencedlg:MasterDefaultHelp').getString;
            masterDefaultHelp.Type = 'text';
            masterDefaultHelp.Italic = 0;
            masterDefaultHelp.WordWrap = true;
            masterDefaultHelp.RowSpan = [row row];
            masterDefaultHelp.ColSpan = [1 3];
            masterDefaultHelp.Visible = obj.Profile.MasterUseDefault;

            % container
            masterContainer.Type = 'group'; % can be 'panel', in which case, case use .Flat = true
            masterContainer.Name = message('robotics:robotslgazebo:preferencedlg:GazeboPlugin').getString;
            masterContainer.Flat = false;
            masterContainer.LayoutGrid = [row 3]; % [numrows numcolumns]
            masterContainer.ColStretch = [2 2 1];
            masterContainer.Items = {masterDefaultHelp, masterHostLabel, masterHostSelection, masterHostNameLabel,...
                                masterHostName, masterPortNumLabel, masterPortNum, masterTestBtn};
            masterContainer.Visible = true;

            %% Co-simulation dialog
            row = 1;
            
            simTimeoutLabel.Name = message('robotics:robotslgazebo:preferencedlg:SimulationTimeout').getString;
            simTimeoutLabel.Type  = 'text';
            simTimeoutLabel.Alignment  = 7;
            simTimeoutLabel.RowSpan = [row row];
            simTimeoutLabel.ColSpan = [1 1];
            simTimeoutLabel.Visible = true;

            simTimeout.Name = '';
            simTimeout.Type = 'edit';
            simTimeout.Value = num2str(obj.Profile.SimulationTimeout);
            simTimeout.RowSpan = [row row];
            simTimeout.ColSpan = [2 2];
            simTimeout.Alignment = 0; % top-left
            simTimeout.Tag = obj.SimulationTimoutTag;
            simTimeout.ObjectMethod = 'simTimeoutChanged'; % call method on UDD source object
            simTimeout.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            simTimeout.ArgDataTypes = {'handle', 'string', 'mxArray'};
            simTimeout.Visible = true;

            % container
            coSimContainer.Type = 'group'; % can be 'panel', in which case, case use .Flat = true
            coSimContainer.Name = message('robotics:robotslgazebo:preferencedlg:CoSimSetup').getString;
            coSimContainer.Flat = false;
            coSimContainer.LayoutGrid = [row 3]; % [numrows numcolumns]
            coSimContainer.ColStretch = [2 2 1];
            coSimContainer.Items = {simTimeoutLabel, simTimeout};
            coSimContainer.Visible = true;

            %% Main Dialog

            helptext.Name = message('robotics:robotslgazebo:preferencedlg:HelpText').getString;
            helptext.Type  = 'text';
            helptext.WordWrap = true;
            helptext.RowSpan = [1 1];
            helptext.ColSpan = [1 2];
            helptext.Tag = obj.GazeboPreferenceHelpTag;
            helptext.Visible = true;

            topLevelContainer.Type = 'group'; % can be 'panel', in which case, case use .Flat = true
            topLevelContainer.Name = '';
            topLevelContainer.Items = {helptext, masterContainer, coSimContainer};
            topLevelContainer.Visible = true;

            %% Main dialog struct

            dlgstruct.DialogTitle = message('robotics:robotslgazebo:preferencedlg:DialogTitle').getString;
            dlgstruct.HelpMethod = 'helpview';
            dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboPacer'};
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            %Make the dialog non-modal like other dialogs
            %there will be only one dialog
            dlgstruct.Sticky = false;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {topLevelContainer};
            dlgstruct.DialogTag = obj.getDialogTag();
        end
    end

    %%
    methods(Static)
        function launch()
        % Convenience function for opening the dialog
            robotics.slcore.internal.dlg.PreferenceSpecifierUtil.launch(...
                'robotics.slgazebo.internal.dlg.GazeboPreferenceSpecifier', ...
                robotics.slgazebo.internal.dlg.GazeboPreferenceSpecifier.getDialogTag());
        end

        function tag = getDialogTag()
            tag = 'slgazebo_preference';
        end

    end

end
