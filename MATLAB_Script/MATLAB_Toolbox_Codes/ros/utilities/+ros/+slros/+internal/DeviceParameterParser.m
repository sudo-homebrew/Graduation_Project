classdef DeviceParameterParser < handle
%This class is for internal use only. It may be removed in the future.

%DeviceParameterParser Class for parsing device parameters

%   Copyright 2016-2020 The MathWorks, Inc.

    methods
        function [validHost, validSSHPort] = validateDeviceAddress(obj, deviceAddress, funcName, varName)
        %validateDeviceAddress Validate the IP address or host name of device

        % Do some basic validation. The input needs to be a character
        % vector.
            validateattributes(deviceAddress, {'char'}, {'nonempty', 'row'}, funcName, varName);

            % Return default SSH port by default
            validSSHPort = ros.codertarget.internal.DeviceParameters.DefaultSSHPort;

            % Parse input and try to extract SSH port number
            hostString = string(deviceAddress);
            hostElements = hostString.split(':');
            switch length(hostElements)
              case 1
                % No port number is specified
                host = char(hostElements);
              case 2
                % Interpret last element as port number
                host = char(hostElements(1));
                sshPort = str2double(hostElements(2));
                if isnan(sshPort)
                    error(message('ros:slros:deviceparams:PortNotNumeric', char(hostElements(2))));
                end
                validSSHPort = obj.validateSSHPort(sshPort, funcName, 'sshPort');
              otherwise
                % There are more than one colon characters in the host
                % name. That is invalid.
                error(message('ros:mlros:util:HostnameInvalid', deviceAddress));
            end

            % Validate that host is valid
            obj.validateHostname(host, funcName, varName);

            % Return valid host
            validHost = host;
        end

        function validateHostname(~, hostname, funcName, varName)
        %validateHostname Validate a hostname

            validateattributes(hostname, {'char'}, {'nonempty', 'row'}, funcName, varName);

            if ~ros.internal.Net.isValidHost(hostname)
                error(message('ros:mlros:util:HostnameInvalid', hostname));
            end
        end

        function validSSHPort = validateSSHPort(~, sshPort, funcName, varName)
        %validateSSHPort Validate SSH port
        %   A port is represented by an unsigned 16-bit number, which
        %   limits the range of valid ports.
        %   See http://www.iana.org/assignments/service-names-port-numbers/service-names-port-numbers.xhtml

            validateattributes(sshPort, {'numeric'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', 'finite', '>=', 0, '<=', 2^16 - 1}, funcName, varName)

            % Always return the port as double value
            validSSHPort = double(sshPort);
        end

        function validUsername = validateUsername(~, username, funcName, varName)
        %validateUsername Validate the username for device login
        %   Linux is pretty forgiving when it comes to user name
        %   syntax, so I am only verifying that this input is a
        %   character vector.

            validateattributes(username, {'char'}, {'nonempty', 'row'}, funcName, varName);
            validUsername = username;
        end

        function validPassword = validatePassword(obj, password, funcName, varName)
        %validatePassword Validate the password for device login
        %   Linux is pretty forgiving when it comes to password
        %   syntax, so I am only verifying that this input is a
        %   character vector.

            attributes = obj.allowEmptyStringAttributes(password);

            validateattributes(password, {'char'}, attributes, funcName, varName);
            validPassword = password;
        end

        function validROSFolder = validateROSFolder(~, rosFolder, funcName, varName)
        %validateROSFolder Validate the ROS installation folder
        %   Note that this is only a syntactic check. This function
        %   does not verify the existence of the folder on the target
        %   device.

            validateattributes(rosFolder, {'char'}, {'nonempty', 'row'}, funcName, varName);
            validROSFolder = rosFolder;
        end

        function validROS2Folder = validateROS2Folder(~, ros2Folder, funcName, varName)
        %validateROSFolder Validate the ROS 2 installation folder name
        %   Note that this is only a syntactic check. This function
        %   does not verify the existence of the folder on the target
        %   device.

            validateattributes(ros2Folder, {'char'}, {'nonempty', 'row'}, funcName, varName);
            validROS2Folder = ros2Folder;
        end
        
        function validCatkinWs = validateCatkinWorkspace(~, catkinWs, funcName, varName)
        %validateCatkinWorkspace Validate the Catkin workspace folder
        %   Note that this is only a syntactic check. This function
        %   does not verify the existence of the folder on the target
        %   device.

            validateattributes(catkinWs, {'char'}, {'nonempty', 'row'}, funcName, varName);
            validCatkinWs = catkinWs;
        end
        
        function validROS2Ws = validateROS2Workspace(~, ros2Ws, funcName, varName)
        %validateROS2Workspace Validate the ROS 2 workspace folder
        %   Note that this is only a syntactic check. This function
        %   does not verify the existence of the folder on the target
        %   device.
            validateattributes(ros2Ws, {'char'}, {'nonempty', 'row'}, funcName, varName);
            validROS2Ws = ros2Ws;
        end
    end

    methods (Static, Access = private)
        function attributes = allowEmptyStringAttributes(arg)
            if isempty(arg)
                attributes = {};
            else
                attributes = {'nonempty', 'row'};
            end
        end
    end
end
