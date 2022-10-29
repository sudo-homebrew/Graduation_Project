classdef DeviceParameterSpecifier < handle
%This class is for internal use only. It may be removed in the future.

%  DeviceParameterSpecifier opens a DDG dialog that lets the user
%  specify the parameters for connecting to a ROS device, e.g. user
%  name, password, device address.
%
%  Example:
%    userDlg = ros.slros.internal.dlg.DeviceParameterSpecifier
%    userDlg.openDialog

%   Copyright 2016-2020 The MathWorks, Inc.

    properties (Access = private)
        %Parser - The parsing helper object
        Parser

        %CloseFcnHandle - The user-specified function handle called when the dialog closes
        %   This function handle is invoked when the user clicks "OK" or
        %   "Cancel".
        CloseFcnHandle = function_handle.empty

        %Hostname - The hostname or IP address of the device
        %   This is part of the 'DeviceAddress' property.
        Hostname

        %SSHPort - SSH port used to connect to the device (default is 22)
        %   This is part of the 'DeviceAddress' property.
        SSHPort

        %IsSticky - Is dialog sticky with respect to other DDG windows
        %   If this is set to true, the user will not be able to interact
        %   with any other DDG dialogs or the model explorer until he
        %   dismisses the sticky dialog.
        IsSticky = false

        %PreventCloseCallback - If TRUE, ignore callbacks received through the dialog CloseMethod
        %   Since we are using custom buttons in this dialog, we are
        %   handling the dialog closing ourselves, so this flag gives us
        %   more control.
        PreventCloseCallback = false
    end

    properties (SetObservable = true)
        %ModelName - The name of the Simulink model associated with this device connection
        ModelName = ''

        %DeviceAddress - Contents of the device address edit field
        %   Note that device addresses can also contain the SSH port, e.g.
        %   '192.168.1.1:40763'. The individual elements are stored in the
        %   'Hostname' and 'SSHPort' properties.
        DeviceAddress {matlab.internal.validation.mustBeASCIICharRowVector(DeviceAddress, 'DeviceAddress')} = char.empty

        %Username - Contents of the user name edit field
        Username {matlab.internal.validation.mustBeASCIICharRowVector(Username, 'Username')} = char.empty

        %Password - Contents of the password edit field
        Password {matlab.internal.validation.mustBeASCIICharRowVector(Password, 'Password')} = char.empty

        %ROSInstallFolder - Contents of the ROS folder edit field
        ROSInstallFolder {matlab.internal.validation.mustBeASCIICharRowVector(ROSInstallFolder, 'ROSInstallFolder')} = char.empty

        %CatkinWorkspace - Contents of the Catkin workspace edit field
        CatkinWorkspace {matlab.internal.validation.mustBeASCIICharRowVector(CatkinWorkspace, 'CatkinWorkspace')} = char.empty

        %ROS2InstallFolder - Contents of the ROS 2 folder edit field
        ROS2InstallFolder {matlab.internal.validation.mustBeASCIICharRowVector(ROS2InstallFolder, 'ROS2InstallFolder')} = char.empty

        %ROS2Workspace - Contents of the Catkin workspace edit field
        ROS2Workspace {matlab.internal.validation.mustBeASCIICharRowVector(ROS2Workspace, 'ROS2Workspace')} = char.empty

        %RememberPassword - State of the "Remember my password" checkbox
        RememberPassword (1,1) logical

    end

    properties (Dependent, SetAccess = private)
        %DialogTag - Unique tag for dialog
        %   This can be used to find the dialog with the findDDGByTag
        %   function.
        DialogTag
    end

    methods (Static)
        function deviceParamDlg = openDialogForModel(modelName, varargin)
        %openDialogForModel Convenience function for opening a dialog for a Simulink model
            [deviceParamDlg, dlg] = ros.slros.internal.dlg.DeviceParameterSpecifier.getCurrentInstance(modelName);
            if isempty(deviceParamDlg)
                deviceParamDlg = ros.slros.internal.dlg.DeviceParameterSpecifier;
                deviceParamDlg.ModelName = modelName;
                deviceParamDlg.openDialog(varargin{:});
            else
                dlg.show();
            end
        end

        function [out, dlg] = getCurrentInstance(modelName)
        %getCurrentInstance Current instance of open dialog window.
        %    This is used by the 'fixit' functionality. The function
        %    returns [] if the dialog window is not open.
            tag = ros.slros.internal.dlg.DeviceParameterSpecifier.getDialogTag(modelName);
            dlgs = findDDGByTag(tag);
            if ~isempty(dlgs)
                dlg = dlgs(end);
                out = dlg.getDialogSource;
            else
                out = [];
                dlg = [];
            end
        end

        function tag = getDialogTag(modelName)
        %getDialogTag Get dialog tag corresponding to a Simulink model.
        %    Return tag for a global instance if no model is specified.
            base = ros.slros.internal.dlg.DeviceParameterSpecifier.DialogTagBase;
            if isempty(modelName)
                tag = base;
            else
                tag = [base, '_', modelName];
            end
        end
        
        function setROS2InstallFolder(modelName, ros2Folder)
        %setROS2InstallFolder Set ROS 2 install folder
        %    If the dialog is open for the given model, it will
        %    set the dialog's folder. Otherwise, it will set the global
        %    DeviceParameters folder.
            dlg = ros.slros.internal.dlg.DeviceParameterSpecifier.getCurrentInstance(modelName);
            if ~isempty(dlg)
                % Set for dialog if it exists and allow user to accept or reject
                dlg.ROS2InstallFolder = ros2Folder;
            else
                % Set globally otherwise
                ros.codertarget.internal.DeviceParameters.setROS2InstallFolder(ros2Folder);
            end
        end
        
        function setROSInstallFolder(modelName, rosFolder)
        %setROSInstallFolder Set ROS install folder for a specified
        %    model. If the dialog is open for the given model, it will
        %    set the dialog's folder. Otherwise, it will set the global
        %    DeviceParameters folder.
            dlg = ros.slros.internal.dlg.DeviceParameterSpecifier.getCurrentInstance(modelName);
            if ~isempty(dlg)
                % Set for dialog if it exists and allow user to accept or reject
                dlg.ROSInstallFolder = rosFolder;
            else
                % Set globally otherwise
                ros.codertarget.internal.DeviceParameters.setROSInstallFolder(rosFolder);
            end
        end
    end

    methods
        function obj = DeviceParameterSpecifier
        %DeviceParameterSpecifier Standard constructor

            obj.Parser = ros.slros.internal.DeviceParameterParser;
        end

        function tag = get.DialogTag(obj)
            tag = ros.slros.internal.dlg.DeviceParameterSpecifier.getDialogTag(obj.ModelName);
        end

        function dlg = openDialog(obj, closeFcnHandle, isSticky)
        %openDialog Open the dialog window
        %   openDialog(OBJ) opens the dialog window
        %
        %   openDialog(OBJ, CLOSEFCNHANDLE) allows you to specify a
        %   function handle that will be called when the dialog is
        %   closed (user clicks OK or Cancel).
        %   CLOSEFCNHANDLE is a function handle that takes 6 arguments,
        %   e.g.
        %
        %   closeFcn(closeAction, deviceAddress, hostname, sshport, username, password, rosfolder, catkinws)
        %      closeAction - Either 'ok' or 'cancel' to indicate what button the user pressed
        %      deviceAddress - Last value for device address (combination of hostname and sshport)
        %      hostname - Hostname or IP address
        %      sshport - SSH port used for connection
        %      username - Last value for username
        %      password - Last value for password
        %      rosFolder - Last value for ROS folder
        %      catkinWs - Last value for Catkin workspace
        %
        %   openDialog(OBJ, CLOSEFCNHANDLE, ISSTICKY) lets you indicate
        %   if the dialog should be sticky (modal) or not. ISSTICKY is
        %   a logical scalar.

            if exist('closeFcnHandle', 'var')
                validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
                obj.CloseFcnHandle = closeFcnHandle;
            else
                obj.CloseFcnHandle = function_handle.empty;
            end

            if exist('isSticky', 'var')
                validateattributes(isSticky, {'logical'}, {'scalar'});
                obj.IsSticky = isSticky;
            end

            % Reset close callback handling
            obj.PreventCloseCallback = false;

            % Initialize the data from the MATLAB preferences
            obj.Hostname = ros.codertarget.internal.DeviceParameters.getHostname;
            obj.SSHPort = ros.codertarget.internal.DeviceParameters.getSSHPort;

            % Assign the values of the properties associated with dialog
            % widgets
            obj.DeviceAddress = ros.codertarget.internal.DeviceParameters.getDeviceAddress;
            obj.Username = ros.codertarget.internal.DeviceParameters.getUsername;
            obj.Password = ros.codertarget.internal.DeviceParameters.getPassword;
            obj.RememberPassword = ros.codertarget.internal.DeviceParameters.getSavePassword;
            obj.CatkinWorkspace = ros.codertarget.internal.DeviceParameters.getCatkinWorkspace;
            obj.ROSInstallFolder = ros.codertarget.internal.DeviceParameters.getROSInstallFolder;
            obj.ROS2InstallFolder = ros.codertarget.internal.DeviceParameters.getROS2InstallFolder;
            obj.ROS2Workspace = ros.codertarget.internal.DeviceParameters.getROS2Workspace;
            % Bring up the dialog window
            dlg = DAStudio.Dialog(obj);
        end

        function [isAccepted, hostname, sshPort, username, password, rosFolder, catkinWs,ros2Folder,ros2ws] = ...
                openDialogAndWait(obj)
            %openDialogAndWait Open dialog and wait until the user closes it
            %   This function is blocking. This window is not model
            %   (sticky), because we want the user to interact with the
            %   diagnostic window when testing device parameters.
            %
            %   The function will return the final value of all internal
            %   device parameters.

            % Bring up the dialog window
            dlg = obj.openDialog(@closeDialog, false);

            % Wait for dialog to be closed (closeDialog callback is
            % executed at that point)
            waitfor(dlg);

            % All output values have already been assigned in the callback
            % Note that the device parameters have to be stored in this
            % callback. If we wait until the code gets here, the dialog has
            % already been deleted and the obj properties are not in a
            % valid state anymore.

            function closeDialog(finalaction, ~, hname, ssh, uname, pw, rosdir, ws, ros2dir, wkspc2)

            % Set all output value
                isAccepted = isequal(finalaction, 'ok');
                hostname = hname;
                sshPort = ssh;
                username = uname;
                password = pw;
                rosFolder = rosdir;
                catkinWs = ws;
                ros2Folder = ros2dir;
                ros2ws = wkspc2;
            end
        end
    end

    methods (Hidden)

        function okayButtonPressed(obj, dlg)
        %okayButtonPressed Callback that is executed when the OK button is pressed

            try
                % Always save device address, user name, remember password
                % selection, catkin workspace, and ROS install folder when
                % user clicks "OK
                ros.codertarget.internal.DeviceParameters.setHostname(obj.Hostname);
                ros.codertarget.internal.DeviceParameters.setSSHPort(obj.SSHPort);
                ros.codertarget.internal.DeviceParameters.setUsername(obj.Username);
                ros.codertarget.internal.DeviceParameters.setSavePassword(obj.RememberPassword);
                ros.codertarget.internal.DeviceParameters.setCatkinWorkspace(obj.CatkinWorkspace);
                ros.codertarget.internal.DeviceParameters.setROSInstallFolder(obj.ROSInstallFolder);
                ros.codertarget.internal.DeviceParameters.setROS2InstallFolder(obj.ROS2InstallFolder);
                ros.codertarget.internal.DeviceParameters.setROS2Workspace(obj.ROS2Workspace);

                % Only save password if user selected "Remember my password"
                if obj.RememberPassword
                    ros.codertarget.internal.DeviceParameters.setPassword(obj.Password);
                else
                    % Reset any saved password if the user selects to not save it
                    ros.codertarget.internal.DeviceParameters.setPassword('');
                end

                % Evaluate function handle for closing
                obj.evaluateCloseFcn('ok');

                % Close dialog. To prevent additional callbacks through
                % dlgClose, set a flag.
                obj.PreventCloseCallback = true;
                dlg.delete

            catch ex
                % Convert all errors to warnings. If they are propagated back to
                % DDG, this causes MATLAB to crash

                warning(ex.identifier, '%s', ex.message)
            end
        end


        function cancelButtonPressed(obj, dlg)
        %cancelButtonPressed Callback that is executed when the Cancel button is pressed
            try
                % Evaluate function handle for closing
                obj.evaluateCloseFcn('cancel');

                % Close dialog without applying any changes.
                % To prevent additional callbacks through dlgClose, set a
                % flag.
                obj.PreventCloseCallback = true;
                dlg.delete

            catch ex
                % Convert all errors to warnings. If they are propagated back to
                % DDG, this causes MATLAB to crash

                warning(ex.identifier, '%s', ex.message)
            end
        end

        function testButtonPressed(obj, ~)
        %testButtonPressed Callback that is executed when the Test button is pressed
            try
                % Run a check on all the user settings
                if (isHardwareBoardROS2(obj))
                    tester = ros.slros.internal.diag.DeviceDiagnostics(obj.ModelName,'ros2');
                    tester.runDiagnostics(obj.Hostname, obj.SSHPort, obj.Username, obj.Password, obj.ROS2InstallFolder, obj.ROS2Workspace);
                else
                    tester = ros.slros.internal.diag.DeviceDiagnostics(obj.ModelName,'ros');
                    tester.runDiagnostics(obj.Hostname, obj.SSHPort, obj.Username, obj.Password, obj.ROSInstallFolder, obj.CatkinWorkspace);
                end
            catch ex
                % Convert all errors to warnings. If they are propagated back to
                % DDG, this causes MATLAB to crash
                warning(ex.identifier, '%s', ex.message)
            end
        end

        function dlgClose(obj, closeaction)
        %dlgClose Callback executed when dialog is closed
        %   This callback is executed if the dialog gets deleted or
        %   the user clicks the X to close the dialog.
        %   We are using custom OK and Cancel buttons and in their
        %   callbacks, the dialog is deleted as well. To prevent double
        %   callbacks, use the PreventCloseCallback flag to indicate if
        %   additional actions should be performed in dlgClose.
        %
        %   CLOSEACTION is always 'cancel', since we do not use the
        %   standard OK button.

            try
                if obj.PreventCloseCallback
                    % Do not do anything if this callback was triggered by
                    % our custom OK or Cancel buttons. They already handled
                    % all necessary actions.
                    return;
                end

                % Evaluate function handle for closing
                obj.evaluateCloseFcn(closeaction);

            catch ex
                % Convert all errors to warnings. If they are propagated back to
                % DDG, this causes MATLAB to crash

                warning(ex.identifier, '%s', ex.message)
            end
        end

        function evaluateCloseFcn(obj, closeaction)
        %evaluateCloseFcn Evaluate the CloseFcnHandle callback

            if ~isempty(obj.CloseFcnHandle)
                feval(obj.CloseFcnHandle, closeaction, obj.DeviceAddress, ...
                      obj.Hostname, obj.SSHPort, obj.Username, obj.Password, ...
                      obj.ROSInstallFolder, obj.CatkinWorkspace, ...
                      obj.ROS2InstallFolder, obj.ROS2Workspace);
            end
        end

        function validateDeviceAddress(obj, deviceAddress, argName)
        %validateDeviceAddress Validate the device address string
        %   Also separate the host name / IP address from the SSH port.
        %   Some examples of valid device addresses:
        %      localhost
        %      somehost:225
        %      192.168.2.1
        %      192.168.2.1:40763

            [validHost, validSSHPort] = obj.Parser.validateDeviceAddress(deviceAddress, '', argName);

            obj.Hostname = validHost;
            obj.SSHPort = validSSHPort;
        end
    end

    properties (Constant, Access = ?matlab.unittest.TestCase)
        %% Tags for all widget elements.
        %  Make the tags accessible to unit tests to verify settings.

        DialogTagBase = 'DeviceParameterSpecifierDialog'
        TextHeaderTag = 'textHeader'
        EditDeviceAddressTag = 'editDeviceAddress'
        EditUsernameTag = 'editUsername'
        EditPasswordTag = 'editPassword'
        CheckboxRememberTag = 'checkboxRememberPw'
        EditROSFolderTag = 'editROSFolder'
        EditCatkinWorkspaceTag = 'editCatkinWs'
        EditROS2FolderTag = 'editROS2Folder'
        EditROS2WorkspaceTag = 'editROS2Workspace'
        ButtonHelpTag = 'buttonHelp'
        ButtonOKTag = 'buttonOK'
        ButtonCancelTag = 'buttonCancel'
        ButtonTestTag = 'buttonTest'
        ButtonPanelTag = 'panelButtonSet'
    end

    methods (Hidden)
        function dlgstruct = getDialogSchema(obj)
        %getDialogSchema Specify the DDG schema of the dialog
            [isROS2, modelEmpty] = isHardwareBoardROS2(obj);
            % Widget for header text
            header.Name = message('ros:slros:deviceparams:DialogHeader').getString;
            header.Type = 'text';
            header.Tag = obj.TextHeaderTag;
            header.MinimumSize = [0 50];
            header.RowSpan = [1 1];
            header.ColSpan = [2 2];
            header.HideName = true;
            header.WordWrap = true;

            % Widget for device address
            deviceAddressLabel = message('ros:slros:deviceparams:DeviceAddress').getString;
            deviceAddress.Name = [deviceAddressLabel ': '];
            deviceAddress.Type = 'edit';
            deviceAddress.Tag = obj.EditDeviceAddressTag;
            deviceAddress.RowSpan = [2 2];
            deviceAddress.ColSpan = [2 2];
            deviceAddress.ObjectProperty = 'DeviceAddress';     % Use an object property to store the value of the edit field
            deviceAddress.Mode = true;                          % Update the property in immediate mode
            deviceAddress.ValidationCallback = @(~,~,deviceAddress,~) ...
                obj.validateDeviceAddress(deviceAddress, deviceAddressLabel);
            deviceAddress.ToolTip = message('ros:slros:deviceparams:DeviceAddressTooltip').getString;

            % Widget for username
            usernameLabel = message('ros:slros:deviceparams:Username').getString;
            username.Name = [usernameLabel ': '];
            username.Type = 'edit';
            username.Tag = obj.EditUsernameTag;
            username.RowSpan = [3 3];
            username.ColSpan = [2 2];
            username.ObjectProperty = 'Username';
            username.Mode = true;
            username.ValidationCallback = @(~,~,username,~) obj.Parser.validateUsername(username, '', usernameLabel);
            username.ToolTip = message('ros:slros:deviceparams:UsernameTooltip').getString;

            % Widget for password
            passwordLabel = message('ros:slros:deviceparams:Password').getString;
            password.Name = [passwordLabel ': '];
            password.Type = 'edit';
            password.Tag = obj.EditPasswordTag;
            password.EchoMode = 'password';         % Obfuscate password characters
            password.RowSpan = [4 4];
            password.ColSpan = [2 2];
            password.ObjectProperty = 'Password';
            password.Mode = true;
            password.ValidationCallback = @(~,~,pw,~) obj.Parser.validatePassword(pw, '', passwordLabel);
            password.ToolTip = message('ros:slros:deviceparams:PasswordTooltip').getString;

            
            % Widget for remember password checkbox
            rememberPw.Name = message('ros:slros:deviceparams:RememberPassword').getString;
            rememberPw.Type = 'checkbox';
            rememberPw.Tag = obj.CheckboxRememberTag;
            rememberPw.MinimumSize = [0 50];
            rememberPw.RowSpan = [5 5];
            rememberPw.ColSpan = [2 2];
            rememberPw.ObjectProperty = 'RememberPassword';
            rememberPw.Mode = true;
            rememberPw.ToolTip = message('ros:slros:deviceparams:RememberPasswordTooltip').getString;

            % Widget for Catkin workspace
            catkinWsLabel = message('ros:slros:deviceparams:CatkinWorkspace').getString;
            catkinWs.Name = [catkinWsLabel ': '];
            catkinWs.Type = 'edit';
            catkinWs.Tag = obj.EditCatkinWorkspaceTag;
            catkinWs.RowSpan = [7 7];
            catkinWs.ColSpan = [2 2];
            catkinWs.ObjectProperty = 'CatkinWorkspace';
            catkinWs.Mode = true;
            catkinWs.Visible = ~isROS2 || modelEmpty;
            catkinWs.ValidationCallback = @(~,~,ws,~) obj.Parser.validateCatkinWorkspace(ws, '', catkinWsLabel);
            catkinWs.ToolTip = message('ros:slros:deviceparams:CatkinWorkspaceTooltip').getString;

            % Widget for ROS folder
            rosFolderLabel = message('ros:slros:deviceparams:ROSFolder').getString;
            rosFolder.Name = [rosFolderLabel ': '];
            rosFolder.Type = 'edit';
            rosFolder.Tag = obj.EditROSFolderTag;
            rosFolder.RowSpan = [6 6];
            rosFolder.ColSpan = [2 2];
            rosFolder.ObjectProperty = 'ROSInstallFolder';
            rosFolder.Mode = true;
            rosFolder.Visible = ~isROS2 || modelEmpty;
            rosFolder.ValidationCallback = @(~,~,dir,~) obj.Parser.validateROSFolder(dir, '', rosFolderLabel);
            rosFolder.ToolTip = message('ros:slros:deviceparams:ROSFolderTooltip').getString;

            % Widget for Catkin workspace
            ros2WsLabel = message('ros:slros:deviceparams:ROS2Workspace').getString;
            ros2Ws.Name = [ros2WsLabel ': '];
            ros2Ws.Type = 'edit';
            ros2Ws.Tag = obj.EditROS2WorkspaceTag;
            ros2Ws.RowSpan = [9 9];
            ros2Ws.ColSpan = [2 2];
            ros2Ws.ObjectProperty = 'ROS2Workspace';
            ros2Ws.Mode = true;
            ros2Ws.Visible = isROS2 || modelEmpty;
            ros2Ws.ValidationCallback = @(~,~,ws,~) obj.Parser.validateCatkinWorkspace(ws, '', ros2WsLabel);
            ros2Ws.ToolTip = message('ros:slros:deviceparams:ROS2WorkspaceTooltip').getString;

            % Widget for ROS folder
            ros2FolderLabel = message('ros:slros:deviceparams:ROS2Folder').getString;
            ros2Folder.Name = [ros2FolderLabel ': '];
            ros2Folder.Type = 'edit';
            ros2Folder.Tag = obj.EditROS2FolderTag;
            ros2Folder.RowSpan = [8 8];
            ros2Folder.ColSpan = [2 2];
            ros2Folder.ObjectProperty = 'ROS2InstallFolder';
            ros2Folder.Mode = true;
            ros2Folder.Visible = isROS2 || modelEmpty;
            ros2Folder.ValidationCallback = @(~,~,dir,~) obj.Parser.validateROSFolder(dir, '', ros2FolderLabel);
            ros2Folder.ToolTip = message('ros:slros:deviceparams:ROS2FolderTooltip').getString;

            % Custom help button
            helpButton.Name = message('ros:slros:deviceparams:HelpButton').getString;
            helpButton.Type = 'pushbutton';
            helpButton.MatlabMethod = 'ros.slros.internal.helpview';
            helpButton.MatlabArgs = {'deviceParamsDlg'};  % doc topic id
            helpButton.RowSpan = [1 1];
            helpButton.ColSpan = [4 4];
            helpButton.Tag = obj.ButtonHelpTag;

            % Custom test button
            testButton.Name = message('ros:slros:deviceparams:TestButton').getString;
            testButton.Type = 'pushbutton';
            testButton.RowSpan = [1 1];
            testButton.ColSpan = [3 3];
            testButton.Tag = obj.ButtonTestTag;
            testButton.ObjectMethod = 'testButtonPressed';
            testButton.MethodArgs = {'%dialog'}; % object handle is implicit first arg
            testButton.ArgDataTypes = {'handle'}; % 'handle' is type of %dialog
            testButton.ToolTip = message('ros:slros:deviceparams:TestButtonTooltip').getString;

            % Custom OK button
            okButton.Name = message('ros:slros:deviceparams:OKButton').getString;
            okButton.Type = 'pushbutton';
            okButton.RowSpan = [1 1];
            okButton.ColSpan = [1 1];
            okButton.Tag = obj.ButtonOKTag;
            okButton.ObjectMethod = 'okayButtonPressed';
            okButton.MethodArgs = {'%dialog'}; % object handle is implicit first arg
            okButton.ArgDataTypes = {'handle'}; % 'handle' is type of %dialog

            % Custom Cancel button
            cancelButton.Name = message('ros:slros:deviceparams:CancelButton').getString;
            cancelButton.Type = 'pushbutton';
            cancelButton.RowSpan = [1 1];
            cancelButton.ColSpan = [2 2];
            cancelButton.Tag = obj.ButtonCancelTag;
            cancelButton.ObjectMethod = 'cancelButtonPressed';
            cancelButton.MethodArgs = {'%dialog'}; % object handle is implicit first arg
            cancelButton.ArgDataTypes = {'handle'}; % 'handle' is type of %dialog

            % Construct custom button panel
            % We cannot use the standard StandaloneButtonSet, because we want
            % to add the custom "Test" button to the same panel.
            buttonPanel.Type = 'panel';
            buttonPanel.LayoutGrid = [1 4];
            buttonPanel.Items = {okButton, cancelButton, helpButton, testButton};
            buttonPanel.Tag = obj.ButtonPanelTag;

            % Main dialog structure
            dlgstruct.DialogTitle = message('ros:slros:deviceparams:DialogTitle').getString;
            dlgstruct.DialogTag = obj.DialogTag;
            dlgstruct.LayoutGrid = [7, 3];
            dlgstruct.ColStretch = [1, 10, 1];
            dlgstruct.StandaloneButtonSet = buttonPanel;        % Use custom button panel
            dlgstruct.Items = { header, deviceAddress, username, password, rememberPw, rosFolder, catkinWs, ros2Ws, ros2Folder};
            dlgstruct.Sticky = obj.IsSticky;
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};
        end
        
        function [isROS2, modelEmpty] = isHardwareBoardROS2(obj)
            if ~isempty(obj.ModelName) && bdIsLoaded(obj.ModelName)
                hwBoard = get_param(obj.ModelName, 'HardwareBoard');
                ros2Board = message('ros:slros2:codegen:ui_hwboard').getString;
                isROS2 = strcmp(hwBoard, ros2Board);
                modelEmpty = false;
            else
                isROS2 = false;
                modelEmpty = true;
            end
        end
    end
    methods
        function set.CatkinWorkspace(obj, value)
            obj.CatkinWorkspace = matlab.internal.validation.makeCharRowVector(value);
        end
        
        function set.ROSInstallFolder(obj, value)
            obj.ROSInstallFolder = matlab.internal.validation.makeCharRowVector(value);
        end
 
        function set.ROS2InstallFolder(obj, value)
            obj.ROS2InstallFolder = matlab.internal.validation.makeCharRowVector(value);
        end
        
        function set.ROS2Workspace(obj, value)
            obj.ROS2Workspace = matlab.internal.validation.makeCharRowVector(value);
        end
 
        function set.Password(obj, value)
            obj.Password = matlab.internal.validation.makeCharRowVector(value);
        end
    
        function set.Username(obj, value)
            obj.Username = matlab.internal.validation.makeCharRowVector(value);
        end
        
        function set.DeviceAddress(obj, value)
            obj.DeviceAddress = matlab.internal.validation.makeCharRowVector(value);
        end
    end
end
