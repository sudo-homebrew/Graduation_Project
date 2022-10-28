classdef Core < robotics.core.internal.mixin.Unsaveable & handle
%Core Launches a ROS core
%   The ROS Core (or roscore) encompasses many key components and nodes
%   that are essential for the ROS network. You must have exactly one ROS
%   core running in the ROS network in order for nodes to communicate.
%   This class allows the creation a ROS core in MATLAB. Please note
%   that a connection to an external ROS core is also possible.
%
%   C = ros.Core returns a Core object and launches a ROS core in MATLAB.
%   This ROS core will accept connections on the default port 11311. Note
%   that MATLAB will only allow the creation of one core on any
%   given port and this constructor will display an error if another core is
%   detected on the same port (default port 11311 in this case)
%
%   C = ros.Core(PORT) returns a Core object and launches a ROS core
%   on PORT.
%
%
%   Core properties:
%      Port        - (Read-Only) Network port at which the Master will be listening
%      MasterURI   - (Read-Only) The URI on which the Master can be reached
%
%
%   Example:
%
%      % Launch a ROS core on default URI http://localhost:11311
%      core = ros.Core
%
%      % Launch a ROS core on localhost and port 12000
%      customcore = ros.Core(12000)

%   Copyright 2020 The MathWorks, Inc.

    properties (SetAccess = private)
        %Port - Network port at which the Master will be listening
        %   Default: 11311
        Port
    end

    properties (Dependent, SetAccess = private)
        %MasterURI - The URI on which the Master can be reached
        %   This property can be used by other ROS nodes to contact this
        %   Master.
        MasterURI
    end


    properties (Access = private)
        %Holds the IP Address of Host Machine
        Host

        %Holds the Command Used to Start the ROS Core
        RosCoreCommandName

        %Holds the Command Used to Start the ROS Master
        RosMasterCommandName

        %Holds the Command Used to Start the ROS Out
        RosOutCommandName

        %Hold the PID of ROS Core Command
        RosCorePID
        
        %Master exe name
        RosMasterCmd
    end

    properties (Access = ?matlab.unittest.TestCase)
        %IsCoreStarted - Indicates if this core is running on the
        %particular port or not
        IsCoreStarted = false
    end

    properties (Constant, Access = private)
        %WaitTimeOutForRosout - Wait time in seconds for rosout to start
        WaitTimeForMasterToStart = 200

        %WaitTimeOutForRosout - Wait time in seconds for rosout to start
        WaitTimeForRosoutToStart = 200

        %WaitTimeToCheckMasterConnection - Wait time in seconds to make conenction with Master
        WaitTimeToCheckMasterConnection = 60

        %WaitTimeToRosCoreToDie - Wait time in seconds to make conenction with Master
        WaitTimeToRosCoreToDie = 30
    end

    methods (Access = public)
        function obj = Core(varargin)
        %Core - Create the ROS core object and automatically start it.
        %   The core will be launched on PORT.

            % Parse the input
            parser = getParser(obj);
            parse(parser, varargin{:});
            isMasterCheckDone = logical(parser.Results.IsMasterCheckDone);
            port = double(parser.Results.port);
            if isempty(port)
                port = ros.internal.getDefaultCorePort;
            end
            if port == 0
                % If the port is zero, auto detect the free port.
                port = ros.internal.findOpenCorePort;
            end

            obj.Port = port;

            intfInfo = ros.internal.Net.getAllIPv4Addresses;
            if ~isempty(getenv('ROS_IP'))
                obj.Host = getenv('ROS_IP');
            elseif ~isempty(intfInfo)
                obj.Host = intfInfo(1).ipAddress;
            else
                obj.Host = 'localhost';
            end

            % Try to launch core
            obj.launchCore(isMasterCheckDone);

            % Set ROS_DEFAULT_NODE_PORT so that the new port value can be
            % used by node when the core has been created with port 0
            setenv('ROS_DEFAULT_NODE_PORT', num2str(obj.Port))
            
            function parser = getParser(~)
                % Set up parser
                parser = inputParser;
                addOptional(parser, 'port', ros.internal.getDefaultCorePort, ...
                            @(x) validateattributes(x, ...
                                                    {'numeric'},{'integer','nonnegative','<=',65535}));
                addParameter(parser, 'IsMasterCheckDone', false, ...
                            @(x) validateattributes(x, {'logical', 'numeric'}, {'scalar'}));
            end
        end

        function delete(obj)
        %DELETE Shut down ROS core
        %   DELETE(OBJ) shuts down the ROS core object OBJ and stops
        %   the associated ROS master.

            try
                obj.stop();
            catch
                warning(message('ros:mlros:core:ShutdownError'));
            end

        end
    end


    methods
        function uri = get.MasterURI(obj)
            uri = ['http://' obj.Host ':',num2str(obj.Port)];
        end
    end

    methods (Access = private)
        function launchCore(obj,isMasterCheckDone)
        %launchCore Launch a core object
            obj.IsCoreStarted = false;

            % If the Core is not started start the core.
            % When master check is done and it is already verified that the
            % core is not started, no need to check it again.
            if isMasterCheckDone || ...
               ros.internal.NetworkIntrospection.isMasterReachable(obj.MasterURI) == false
                start(obj);
                obj.IsCoreStarted = true;
            elseif ros.internal.NetworkIntrospection.isValidMaster(obj.MasterURI)
                error(message('ros:mlros:core:AlreadyRunning', obj.Port));
            else
                error(message('ros:mlros:core:InvalidPort', obj.Port));
            end

        end

        function start(obj)
        %start - (Re)Start the core
        %   Gives you the ability to re-start the core after it has
        %   been stopped. The function will throw an error if this (or
        %   another core) is already running on the same port.

            setenv('ROS_MASTER_IP_ADDRESS',obj.Host);
            setenv('ROS_MASTER_IP_PORT',num2str(obj.Port));
            setenv('ROS_MATLAB_PID',num2str(feature('getpid')));
            disp(getString(message('ros:mlros:core:InitializeROSCore')));
            if ispc
                obj.RosMasterCmd = 'rosmaster';
                roscorecmd = ['title ' obj.RosMasterCmd ' && ' obj.RosMasterCmd ' --core -p ' num2str(obj.Port) ' -w 3 &'];
            elseif ismac
                % rosout process is not getting launched using roscore
                % python. So manually launching the rosout process.
                % rosoutcmd = '$CATKIN_PREFIX_PATH/lib/rosout/rosout __name:=rosout';
                % [~, ~] = ros.internal.runroscmd(rosoutcmd);
                obj.RosMasterCmd = ['rosmaster --core -p ' num2str(obj.Port) ' -w 3'];
                roscorecmd = [obj.RosMasterCmd ' &'];
            else
                obj.RosMasterCmd = ['rosmaster --core -p ' num2str(obj.Port) ' -w 3'];
                roscorecmd = [obj.RosMasterCmd ' &'];
            end

            % The return and result are always zero as command is running
            % in background.
            [~, ~] = ros.internal.runroscmd(roscorecmd);

            % Pyhon roscore launch is taking much time in Windows.
            % Wait until the rosout process is launched.
            if ispc
                obj.RosCoreCommandName = [obj.RosMasterCmd '  - ' obj.RosMasterCmd '  --core -p ' num2str(obj.Port) ' -w 3'];
                isAdminMode = System.Security.Principal.WindowsPrincipal(...
                        System.Security.Principal.WindowsIdentity.GetCurrent()).IsInRole(...
                        System.Security.Principal.WindowsBuiltInRole.Administrator);
                if isAdminMode
                    obj.RosCoreCommandName = ['Administrator:  ' obj.RosCoreCommandName];
                end
                obj.RosMasterCommandName = obj.RosMasterCmd;
                obj.RosOutCommandName = 'rosout';
            else
                obj.RosCoreCommandName = [fullfile(matlabroot,'bin',computer('arch')) ' ' obj.RosMasterCmd];
                obj.RosMasterCommandName = ['rosmaster --core -p ' num2str(obj.Port)];
                obj.RosOutCommandName = fullfile(matlabroot,'sys', 'ros1', computer('arch'),'ros1','lib', 'rosout');
            end

            ros.internal.waitForMasterToStart(obj.MasterURI, obj.WaitTimeForRosoutToStart);
            %ros.internal.waitForProcessToStart(obj.RosOutCommandName, obj.WaitTimeForRosoutToStart);

            obj.RosCorePID = ros.internal.getProcessPID(obj.RosCoreCommandName);
        end

        function stop(obj)
        %stop - Stop the ROS core.
        %   Use this function to shutdown the core without having to
        %   delete the object.
            if obj.IsCoreStarted

                if ispc
                    % On Windows, Kill the comamnd window which launched the process.
                    [killStatus, ~]= ros.internal.killROSProcess(obj.RosMasterCmd,obj.RosCoreCommandName);

                    if isequal(killStatus,false)
                        warning(message('ros:mlros:core:ShutdownError'));
                        %fprintf('Output of kill command: %s', result);
                    end
                else
                    % On other platforms, kill the rosmaster process
                    [status, ~] = ros.internal.killProcessByPID(obj.RosCorePID);

                    % If the kill is not success.
                    if ~isequal(status,false)
                        warning(message('ros:mlros:core:ShutdownError'));
                        %fprintf('Output of kill command: %s', result);
                    end
                end
                ros.internal.waitForProcessToDie(obj.RosCoreCommandName,obj.WaitTimeToRosCoreToDie);

                % Ensure that shutdown was successful
                ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                             ~ros.internal.NetworkIntrospection.isMasterReachable(obj.MasterURI, 0.5), ...
                                                             5 );
            end
        end
    end
end
