classdef rosdevice < robotics.core.internal.mixin.Unsaveable & ...
        ros.codertarget.internal.ROSDeviceInterface
    %ROSDEVICE Connect to remote ROS device
    %
    %   DEVICE = ROSDEVICE(DEVICEADDRESS, USERNAME, PASSWORD) creates a
    %   ROSDEVICE object connected to the ROS device at DEVICEADDRESS. Use
    %   USERNAME and PASSWORD as login credentials. The DEVICEADDRESS is an
    %   IP address such as '192.168.0.10' or a hostname such as
    %   'samplehost.foo.com'.
    %
    %   DEVICE = ROSDEVICE creates a ROSDEVICE object connected
    %   to a ROS device using saved values for DEVICEADDRESS,
    %   USERNAME and PASSWORD.
    %
    %
    %   ROSDEVICE properties:
    %      DeviceAddress   - Hostname or IP address of the ROS device
    %      Username        - Username used to connect
    %      ROSFolder       - Folder where ROS is installed on device
    %      CatkinWorkspace - Catkin folder where models are deployed on device
    %      AvailableNodes  - Nodes that are available to run on device
    %
    %   ROSDEVICE methods:
    %      runNode       - Start ROS node
    %      stopNode      - Stop ROS node
    %      isNodeRunning - Determine if ROS node is running
    %      runCore       - Start ROS core
    %      stopCore      - Stop ROS core
    %      isCoreRunning - Determine if ROS core is running
    %      system        - Execute system command on device
    %      putFile       - Copy file to device
    %      getFile       - Get file from device
    %      deleteFile    - Delete file on device
    %      dir           - List directory contents on device
    %      openShell     - Open interactive command shell to the device
    %
    %
    %   Example:
    %       % Connect to ROS device
    %       device = rosdevice
    %
    %       % Launch a ROS core on the device
    %       runCore(device)
    %
    %       % Display all runnable nodes in the Catkin workspace
    %       device.AvailableNodes
    %
    %       % Run the 'robotROSFeedbackControlExample' node
    %       % This model needs to be deployed to the ROS device
    %       runNode(device, 'robotROSFeedbackControlExample')
    %
    %       % Verify that node is running
    %       isNodeRunning(device, 'robotROSFeedbackControlExample')
    %
    %       % Stop the node
    %       stopNode(device, 'robotROSFeedbackControlExample')
    %
    %       % Shut down the ROS core on the device
    %       stopCore(device)
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    properties (SetAccess = private)
        %DeviceAddress - Hostname or IP address of the Remote device
        %   For example, this can be an IP address or a hostname.
        DeviceAddress = ''
    end
    
    properties (SetAccess = private)
        %Username - Username used to connect to the device
        Username = ''
    end
    
    properties (Dependent, SetAccess = private)
        %AvailableNodes - Nodes that are available to run
        %   This list captures deployed Simulink nodes in the ROS or ROS 2
        %   workspace that are available to run.
        AvailableNodes
    end
    
    properties (Access = {?matlab.unittest.TestCase})
        %Port - SSH port used to connect to the Remote device
        %   Default: 22
        Port = 22
        
        %Password - Password used to connect to the Remote device
        Password = ''
        
        %Parser - Parser object for user inputs
        Parser
        
        %Diagnostic - Diagnostic helper object
        Diagnostic
    end
    
    properties
        %ROSFolder - Folder where ROS is installed
        %   This is a folder name on the ROS device. By default, this is
        %   initialized from the stored settings.
        ROSFolder
        
        %CatkinWorkspace - Catkin workspace where models are deployed
        %   This is a folder name on the ROS device. By default, this
        %   is initialized from the stored settings.
        CatkinWorkspace
    end
    
    properties (Access = ?ros.slros.internal.InternalAccess)
        SystemExecutor
        CoreExecutor
        NodeExecutor
    end
    
    methods
        function obj = rosdevice(varargin)
            %ROSDEVICE Construct an instance of this class

            import ros.codertarget.internal.*
            initDeviceParameters(obj,varargin{:});
            
            % Create executors
            obj.SystemExecutor = createSystemExecutor(obj.DeviceAddress,...
                obj.Username,obj.Password,obj.Port);
            obj.CoreExecutor = createCoreExecutor(obj.DeviceAddress,...
                obj.SystemExecutor);
            obj.NodeExecutor = createNodeExecutor(obj.DeviceAddress,...
                obj.SystemExecutor,'ros');     
                        
            % Automatically set ROSFolder and CatkinWorkspace variables
            if isequal(obj.DeviceAddress,'localhost')
                obj.ROSFolder = obj.SystemExecutor.fullfile(matlabroot,...
                    'sys','ros1',computer('arch'),'ros1');
                obj.CatkinWorkspace = pwd;
            else
                % Derive ROS folder and CatkinWorkspace from deviceParams
                deviceParams = ros.codertarget.internal.DeviceParameters;
                obj.ROSFolder = i_handleSpaces(deviceParams.getROSInstallFolder);
                obj.CatkinWorkspace = i_handleSpaces(deviceParams.getCatkinWorkspace);
            end
        end
        
        %% Getter and setter methods
        function nodeList = get.AvailableNodes(obj)
            % Return list of available nodes
            nodeList = getAvailableNodes(obj.NodeExecutor,getWorkspaceFolder(obj));
        end
        
        function set.ROSFolder(obj,rosFolder)
            % This class is unsaveable. Hence accessing another property
            % in the set function of ROSFolder is ok
            validateattributes(rosFolder,{'char'},{'nonempty','row'},'','ROSFolder');
            obj.ROSFolder = rosFolder;
        end
        
        %% Implement system calls through system executor
        function output = system(obj,varargin) %command,sudo)
            %SYSTEM execute command on device and return output
            output = system(obj.SystemExecutor,varargin{:});
        end
        
        function putFile(obj,varargin)
            % PUTFILE Copy localFile on the host computer to the remoteFile
            % on device
            %
            % The remoteFile argument is optional. If not specified, the
            % localFile is copied to the user's home directory.
            %
            % See also dir, getFile and putFile.
            putFile(obj.SystemExecutor,varargin{:})
        end
        
        function getFile(obj,varargin)
            % GETFILE Copy remoteFile on device to the localFile on the
            % host computer
            getFile(obj.SystemExecutor,varargin{:});
        end
        
        function deleteFile(obj,varargin)
            % DELETEFILE Delete file on device
            deleteFile(obj.SystemExecutor,varargin{:});
        end
        
        function d = dir(obj,varargin)
            %DIR List contents of a directory
            d = dir(obj.SystemExecutor,varargin{:});
        end
        
        function openShell(obj)
            %OPENSHELL Open a command terminal
            openShell(obj.SystemExecutor);
        end
        
        %% Use core executor to implement core interface
        function runCore(obj)
            %runCore Start ROS core on device
            %   runCore(DEVICE) starts the ROS core on the device
            %   connected through the DEVICE object. The ROS master uses
            %   the default port number of 11311.
            %
            %   The version of the ROS core that is started is
            %   determined by the following heuristics:
            %   1. If ROSFolder contains a valid ROS installation folder,
            %      start the ROS core from there
            %   2. If the CatkinWorkspace is a valid workspace, start the
            %      ROS core based on the ROS installation that is associated
            %      with this workspace.
            %
            %   See also stopCore.
            
            % Check if roscore is running. Display an error if it is.
            % Note that we cannot determine on which port the existing
            % roscore is running, so only allow one instance.
            runCore(obj.CoreExecutor,obj.ROSFolder,obj.CatkinWorkspace);
        end
        
        function stopCore(obj)
            %stopCore Stop ROS core on device
            %   stopCore(DEVICE) stops the ROS core on the device
            %   connected through the DEVICE object.
            %   If multiple roscore processes are running on the device,
            %   this function stops all of them.
            %
            %   If the core is not running, this function returns right
            %   away.
            %
            %   See also runCore.
            
            % Run command with 'sudo' if user has administrative
            % privileges. This enables killing roscore processes launched
            % by other users.
            
            % rosmaster and rosout might have been created by "roslaunch".
            % In that case, don't kill roslaunch, since that might affect
            % other running ROS nodes.
            stopCore(obj.CoreExecutor);
        end
        
        function isRunning = isCoreRunning(obj)
            %isCoreRunning Determine if ROS core is running on device
            %   ISRUNNING = isCoreRunning(DEVICE) returns TRUE if the ROS
            %   core is running on the device connected through the DEVICE
            %   object. The function returns FALSE if the core is not
            %   running on the device.
            
            % There are two ways in which a ROS core could be active
            % - The user called roscore, so the following processes are
            %   active: roscore, rosmaster, rosout.
            % - The user called roslaunch and no other ROS core was
            %   running. In that case, the following processes are active:
            %   roslaunch, rosmaster, rosout.
            isRunning = isCoreRunning(obj.CoreExecutor);
        end
        
        %% Use node executor to implement node interface
        function runNode(obj,modelName,rosMasterURI,nodeHost)
            %runNode Start ROS node on device
            %   runNode(DEVICE, MODELNAME) starts the ROS node associated
            %   with the Simulink model with name MODELNAME on the connected
            %   DEVICE. The ROS node needs to be deployed in the Catkin workspace
            %   specified in the 'CatkinWorkspace' property. The node connects
            %   to the same ROS master that MATLAB is connected to and advertises
            %   its address as the property value 'DeviceAddress'.
            %
            %   runNode(DEVICE, MODELNAME, ROSMASTERURI) runs the node and
            %   connects it to the ROS master running at ROSMASTERURI.
            %
            %   runNode(DEVICE, MODELNAME, ROSMASTERURI, NODEHOST) runs the
            %   node and connects it to ROSMASTERURI. The node advertises
            %   its address as the hostname or IP address given in
            %   NODEHOST.
            %
            %
            %   Example:
            %       device = rosdevice
            %
            %       % Run the 'robotROSFeedbackControlExample' node
            %       % This model needs to be deployed to the ROS device
            %       runNode(device, 'robotROSFeedbackControlExample')
            %
            %       % Stop the node
            %       stopNode(device, 'robotROSFeedbackControlExample')
            %
            %       % Run the node again and connect to the ROS
            %       % Master at IP 192.168.1.1. The node should advertise
            %       % its address as 192.168.1.20
            %       runNode(device, 'robotROSFeedbackControlExample', 'http://192.168.1.1:11311', '192.168.1.20')
            %
            %   See also stopNode.
            narginchk(2, 4);
            
            % Parse inputs
            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'runNode', 'modelName');
            
            % If node is already running, don't do anything
            if obj.isNodeRunning(modelName)
                disp(message('ros:slros:rosdevice:NodeAlreadyRunning', modelName).getString);
                return;
            end
            
            if nargin < 3
                % Use default MasterURI
                rosMaster = ros.slros.internal.sim.ROSMaster;
                verifyReachable(rosMaster);
                rosMasterURI = rosMaster.MasterURI;
            else
                % Parse user input. The function displays an error if
                % the URI is not valid.
                rosMasterURI = ros.internal.Net.canonicalizeURI(rosMasterURI);
            end
            
            if nargin < 4
                nodeHost = '';
            else
                % Parse user input. The function displays an error if
                % the hostname or IP address is not valid.             
                if ~ros.internal.Net.isValidHost(nodeHost)
                    error(message('ros:mlros:util:HostnameInvalid',nodeHost));
                end
            end
            runNode(obj.NodeExecutor,modelName,obj.CatkinWorkspace,rosMasterURI,nodeHost);
        end
        
        function stopNode(obj,modelName)
            %stopNode Stop the node on device
            %   stopNode(DEVICE, MODELNAME) stops the node associated
            %   with the Simulink model with name MODELNAME on the connected
            %   DEVICE.
            %
            %   If the node is not running, this function returns right
            %   away.
            %
            %
            %   Example:
            %       device = rosdevice
            %
            %       % Run the 'exampleModel' node
            %       % This model needs to be deployed to the ROS device.
            %       runNode(device, 'exampleModel')
            %
            %       % Stop the node
            %       stopNode(device, 'exampleModel')
            %
            %       % Calling stop again has no effect
            %       stopNode(device, 'exampleModel')
            %
            %   See also runNode.
            stopNode(obj.NodeExecutor,modelName);
        end
        
        function isRunning = isNodeRunning(obj,modelName)
            %isNodeRunning Determine if ROS node is running on device
            %   ISRUNNING = isNodeRunning(DEVICE, MODELNAME) returns TRUE
            %   if the ROS node associated with the Simulink model with
            %   name MODELNAME is running on the DEVICE.
            %   The function returns FALSE if the node is not
            %   running on the device.
            %
            % Se also runNode, stopNode.
            validateattributes(modelName,{'char'},{'nonempty','row'},...
                'isNodeRunning','modelName');
            isRunning = isNodeRunning(obj.NodeExecutor,modelName);
        end
    end
    
    methods (Access = protected)
        function ret = getWorkspaceFolder(obj)
            ret = obj.CatkinWorkspace;
        end
        
        function initDeviceParameters(obj,hostname,username,password,port)
            parser = ros.slros.internal.DeviceParameterParser;
            deviceParams = ros.codertarget.internal.DeviceParameters;
            
            % Parse the user input and initialize the object
            % Since all inputs are optional, parse them progressively.
            if nargin < 2
                hostname = deviceParams.getHostname;
                assert(~isempty(hostname),message('ros:slros:rosdevice:InvalidDeviceAddress'));
            else
                % Validate provided host name
                parser.validateHostname(hostname,'rosdevice','hostname');
            end
            obj.DeviceAddress = hostname;
            
            if ~isequal(obj.DeviceAddress,'localhost')
                % Initialize the username
                if nargin < 3
                    username = deviceParams.getUsername;
                    assert(~isempty(username), message('ros:slros:rosdevice:InvalidUsername'));
                else
                    % Validate provided username
                    parser.validateUsername(username,'rosdevice','username');
                end
                obj.Username = username;

                % Initialize the password
                if nargin < 4
                    password = deviceParams.getPassword;
                    assert(~isempty(password),message('ros:slros:rosdevice:InvalidPassword'));
                else
                    % Validate provided password
                    parser.validatePassword(password,'rosdevice','password');
                end
                obj.Password = password;

                % Initialize the SSH port
                if nargin < 5
                    obj.Port = deviceParams.getSSHPort;
                else
                    % Validate provided SSH port
                    obj.Port = parser.validateSSHPort(port,'rosdevice','port');
                end
            end
        end
    end
end

function escapedPath = i_handleSpaces(filePath)
%handleSpaces Handle spaces in file or folder path
%   In the system command that we send over SSH, spaces in the
%   path to a file or a folder need to be escaped with a backslash.

escapedPath = strrep(filePath, ' ', '\ ');

% If the spaces were already escaped, we should undo the
% double-escaping
escapedPath = strrep(escapedPath, '\\ ', '\ ');
end