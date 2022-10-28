classdef DeviceParameters
%This class is for internal use only. It may be removed in the future.

%DeviceParameters Manage ROS device parameters
%   This class stores the default parameters and manages the reading
%   and writing to persistent MATLAB preferences.
%   This class is modeled after the BoardParameters class for generic
%   Linux targets.

%   Copyright 2016-2021 The MathWorks, Inc.

    properties (Constant)
        %PreferenceGroup - The group name for MATLAB preferences
        PreferenceGroup = [ros.slros.internal.Constants.PreferenceGroup '_ROS_Device']

        %% Default values for preferences
        DefaultHostname         = '';
        DefaultUsername         = '';
        DefaultPassword         = '';
        DefaultSavePassword     = false;
        DefaultCatkinWorkspace  = '~/catkin_ws'
        DefaultROS2Workspace    = '~/ros2_ws'
        DefaultSSHPort          =  22;
        DefaultROSInstall       = '/opt/ros/noetic'
        DefaultROS2Install      = '/opt/ros/foxy'
    end

    properties (Constant, Access = ?matlab.unittest.TestCase)
        %% Preference names to store values
        HostnamePrefName         = 'Hostname';
        UsernamePrefName         = 'Username';
        PasswordPrefName         = 'Password';
        SavePasswordPrefName     = 'SavePassword';
        CatkinWorkspacePrefName  = 'CatkinWorkspace';
        ROS2WorkspacePrefName    = 'ROS2Workspace';
        SSHPortPrefName          = 'SSHPort';
        ROSInstallPrefName       = 'ROSInstall';
        ROS2InstallPrefName      = 'ROS2Install';
    end

    properties (Hidden)
        %Parser - The parsing helper object
        Parser
    end

    methods
        function obj = DeviceParameters
        %DeviceParameters Standard constructor

            obj.Parser = ros.slros.internal.DeviceParameterParser;
        end

        function [hostName, sshPort, userName, password, savePassword, catkinWs, rosInstall, ros2Ws, ros2Install] = getDeviceParameters(obj)
        %getDeviceParameters Return all device parameters currently stored in MATLAB preferences

            hostName = obj.getParam('hostname');
            sshPort = obj.getParam('sshport');
            userName = obj.getParam('username');
            password = obj.getParam('password');
            savePassword = obj.getParam('savepassword');
            catkinWs = obj.getParam('catkinws');
            rosInstall = obj.getParam('rosinstall');
            ros2Install = obj.getParam('ros2install');
            ros2Ws = obj.getParam('ros2ws');
        end

        function clearDeviceParameters(obj)
        %clearDeviceParameters Remove device parameters from MATLAB preferences
            obj.removePref(obj.HostnamePrefName);
            obj.removePref(obj.SSHPortPrefName);
            obj.removePref(obj.UsernamePrefName);
            obj.removePref(obj.PasswordPrefName);
            obj.removePref(obj.SavePasswordPrefName);
            obj.removePref(obj.CatkinWorkspacePrefName);
            obj.removePref(obj.ROSInstallPrefName);
            obj.removePref(obj.ROS2InstallPrefName);
            obj.removePref(obj.ROS2WorkspacePrefName);
        end

        function ret = getParam(obj, parameterName)
        %getParam Get parameter with a name from MATLAB preferences

            switch lower(parameterName)
              case {'hostname', 'ipaddress'}
                ret = obj.getPref(obj.HostnamePrefName);
                if ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultHostname;
                end

              case 'username'
                ret = obj.getPref(obj.UsernamePrefName);
                if ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultUsername;
                end

              case 'password'
                ret = obj.getPref(obj.PasswordPrefName);
                if ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultPassword;
                end

              case 'savepassword'
                ret = obj.getPref(obj.SavePasswordPrefName);
                if ~isa(ret, 'logical')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultSavePassword;
                end
              case 'rosinstall'
                ret = obj.getPref(obj.ROSInstallPrefName);
                if ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultROSInstall;
                end

              case 'catkinws'
                ret = obj.getPref(obj.CatkinWorkspacePrefName);
                if  ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultCatkinWorkspace;
                end

              case 'ros2install'
                ret = obj.getPref(obj.ROS2InstallPrefName);
                if ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultROS2Install;
                end

              case 'ros2ws'
                ret = obj.getPref(obj.ROS2WorkspacePrefName);
                if  ~isa(ret, 'char')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultROS2Workspace;
                end

              case 'sshport'
                ret = obj.getPref(obj.SSHPortPrefName);
                if isempty(ret) || ~isa(ret, 'numeric')
                    % MATLAB preference is invalid. Return default value.
                    ret = obj.DefaultSSHPort;
                end

              otherwise
                % Get preference with generic name
                % Note that the parameterName is case-sensitive.
                ret = obj.getPref(['Default' parameterName, 'Pref']);
            end
        end

        function setParam(obj, parameterName, parameterValue)
        %setParam Write parameter with name to MATLAB preferences

            switch lower(parameterName)
              case {'hostname', 'ipaddress'}
                obj.setPref(obj.HostnamePrefName, parameterValue);

              case 'username'
                obj.setPref(obj.UsernamePrefName, parameterValue);

              case 'password'
                obj.setPref(obj.PasswordPrefName, parameterValue);

              case 'savepassword'
                obj.setPref(obj.SavePasswordPrefName, parameterValue);

              case 'catkinws'
                obj.setPref(obj.CatkinWorkspacePrefName, parameterValue);

              case 'rosinstall'
                obj.setPref(obj.ROSInstallPrefName, parameterValue);

              case 'sshport'
                obj.setPref(obj.SSHPortPrefName, parameterValue);

              case 'ros2install'
                obj.setPref(obj.ROS2InstallPrefName, parameterValue);

              case 'ros2ws'
                obj.setPref(obj.ROS2WorkspacePrefName, parameterValue);

              otherwise
                % Set preference with generic name
                % Note that the parameterName is case-sensitive.
                obj.setPref(['Default' parameterName, 'Pref'], ...
                            parameterValue);
            end
        end

        function removeParam(obj, parameterName)
        %removeParam Remove parameter from MATLAB preferences

            switch lower(parameterName)
              case {'hostname', 'ipaddress'}
                obj.removePref(obj.HostnamePrefName);

              case 'username'
                obj.removePref(obj.UsernamePrefName);

              case 'password'
                obj.removePref(obj.PasswordPrefName);

              case 'savepassword'
                obj.removePref(obj.SavePasswordPrefName);

              case 'rosinstall'
                obj.removePref(obj.ROSInstallPrefName);

              case 'catkinws'
                obj.removePref(obj.CatkinWorkspacePrefName);

              case 'sshport'
                obj.removePref(obj.SSHPortPrefName);

              case 'ros2install'
                obj.removePref(obj.ROS2InstallPrefName);

              case 'ros2ws'
                obj.removePref(obj.ROS2WorkspacePrefName);

              otherwise
                % Remove preference with generic name
                % Note that the parameterName is case-sensitive.
                obj.removePref(['Default' parameterName, 'Pref']);
            end
        end
    end

    %% Convenience static functions for retrieving and setting common board parameters
    methods (Static)
        function catkinWs = getCatkinWorkspace
        %getCatkinWorkspace Get value for the Catkin workspace

            deviceParams = ros.codertarget.internal.DeviceParameters;
            catkinWs = deviceParams.getParam('catkinws');
        end

        function setCatkinWorkspace(catkinWs)
        %setCatkinWorkspace Set value for the Catkin workspace

            deviceParams = ros.codertarget.internal.DeviceParameters;

            catkinWs = strtrim(catkinWs);
            if isempty(catkinWs)
                deviceParams.setParam('catkinws', '');
                return;
            end

            deviceParams.Parser.validateCatkinWorkspace(catkinWs, 'setCatkinWorkspace', 'catkinWs');
            deviceParams.setParam('catkinws', catkinWs);
        end

        function deviceAddress = getDeviceAddress
        %getDeviceAddress Get concatenation of host name and IP address

            device = ros.codertarget.internal.DeviceParameters;
            hostname = device.getParam('hostname');
            sshPort = device.getParam('sshport');

            if sshPort == ros.codertarget.internal.DeviceParameters.DefaultSSHPort
                % Do not display SSH port if it is the default
                deviceAddress = hostname;
            else
                deviceAddress = [hostname ':' num2str(sshPort)];
            end
        end

        function hostname = getHostname
        %getDeviceAddress Get IP address or host name of device

            device = ros.codertarget.internal.DeviceParameters;
            hostname = device.getParam('hostname');
        end

        function setHostname(hostname)
        %setHostname Set IP address or host name of device

            deviceParams = ros.codertarget.internal.DeviceParameters;

            hostname = strtrim(hostname);
            if isempty(hostname)
                deviceParams.setParam('hostname','');
                return;
            end

            deviceParams.Parser.validateHostname(hostname, 'setHostname', 'hostname');
            deviceParams.setParam('hostname', hostname);
        end

        function sshPort = getSSHPort
        %getSSHPort Get SSH port of device

            device = ros.codertarget.internal.DeviceParameters;
            sshPort = device.getParam('sshport');
        end

        function setSSHPort(sshPort)
        %setSSHPort Set SSH port of device

            deviceParams = ros.codertarget.internal.DeviceParameters;

            validPort = deviceParams.Parser.validateSSHPort(sshPort, 'setSSHPort', 'sshPort');
            deviceParams.setParam('sshport', validPort);
        end

        function username = getUsername
        %getUsername Get username for login

            deviceParams = ros.codertarget.internal.DeviceParameters;
            username = deviceParams.getParam('username');
        end

        function setUsername(username)
        %setUsername Set username for login

            deviceParams = ros.codertarget.internal.DeviceParameters;

            username = strtrim(username);
            if isempty(username)
                deviceParams.setParam('username','');
                return;
            end

            deviceParams.Parser.validateUsername(username, 'setUsername', 'username');
            deviceParams.setParam('username', username);
        end

        function password = getPassword
        %getPassword Get password for login

            deviceParams = ros.codertarget.internal.DeviceParameters;
            password = deviceParams.getParam('password');

        end

        function setPassword(password)
        %setPassword Set password for login

            deviceParams = ros.codertarget.internal.DeviceParameters;

            password = strtrim(password);
            if isempty(password)
                deviceParams.setParam('password','');
                return;
            end

            deviceParams.Parser.validatePassword(password, 'setPassword', 'password');
            deviceParams.setParam('password', password);
        end

        function savePassword = getSavePassword
        %getSavePassword Get indication if password should be saved

            deviceParams = ros.codertarget.internal.DeviceParameters;
            savePassword = deviceParams.getParam('savepassword');
        end

        function setSavePassword(savePassword)
        %setSavePassword Set indication if password should be saved

            deviceParams = ros.codertarget.internal.DeviceParameters;

            validateattributes(savePassword, {'numeric', 'logical'},{'finite','nonnan','real'}, '', 'SavePassword');
            deviceParams.setParam('savepassword', logical(savePassword));
        end

        function rosFolder = getROSInstallFolder
        %getROSInstallFolder Get root folder of ROS installation

            deviceParams = ros.codertarget.internal.DeviceParameters;
            rosFolder = deviceParams.getParam('rosinstall');

        end

        function setROSInstallFolder(rosFolder)
        %setROSInstallFolder Set root folder of ROS installation

            deviceParams = ros.codertarget.internal.DeviceParameters;

            rosFolder = strtrim(rosFolder);
            if isempty(rosFolder)
                deviceParams.setParam('rosinstall','');
                return;
            end

            deviceParams.Parser.validateROSFolder(rosFolder, 'setROSInstallFolder', 'rosFolder');
            deviceParams.setParam('rosinstall', rosFolder);
        end

        function rosFolder = getROS2InstallFolder
        %getROS2InstallFolder Get root folder of ROS 2 installation

            deviceParams = ros.codertarget.internal.DeviceParameters;
            rosFolder = deviceParams.getParam('ros2install');

        end

        function setROS2InstallFolder(ros2Folder)
        %setROS2InstallFolder Set root folder of ROS 2 installation

            deviceParams = ros.codertarget.internal.DeviceParameters;

            ros2Folder = strtrim(ros2Folder);
            if isempty(ros2Folder)
                deviceParams.setParam('ros2install','');
                return;
            end

            deviceParams.Parser.validateROSFolder(ros2Folder, 'setROSInstallFolder', 'rosFolder');
            deviceParams.setParam('ros2install', ros2Folder);
        end

        function ros2ws = getROS2Workspace()
        %getROS2Workspace Get value for the ROS 2 workspace

            deviceParams = ros.codertarget.internal.DeviceParameters;
            ros2ws = deviceParams.getParam('ros2ws');
        end

        function setROS2Workspace(ros2ws)
        %setROS2Workspace Set value for the ROS 2 workspace

            deviceParams = ros.codertarget.internal.DeviceParameters;

            ros2ws = strtrim(ros2ws);
            if isempty(ros2ws)
                deviceParams.setParam('ros2ws', '');
                return;
            end

            deviceParams.Parser.validateCatkinWorkspace(ros2ws, 'setROS2Workspace', 'ros2ws');
            deviceParams.setParam('ros2ws', ros2ws);
        end

    end


    methods (Access = protected)
        function prefValue = getPref(obj, pref)
        %getPref Read preference value from MATLAB preferences

            prefGroup = obj.PreferenceGroup;
            if ispref(prefGroup, pref)
                prefValue = getpref(prefGroup, pref);
            else
                prefValue = [];
            end
        end

        function setPref(obj, pref, value)
        %setPref Write preference value to MATLAB preferences

            prefGroup = obj.PreferenceGroup;
            setpref(prefGroup, pref, value);
        end

        function removePref(obj, pref)
        %removePref Remove preference from MATLAB preferences

            prefGroup = obj.PreferenceGroup;
            rmpref(prefGroup, pref);
        end
    end
end
