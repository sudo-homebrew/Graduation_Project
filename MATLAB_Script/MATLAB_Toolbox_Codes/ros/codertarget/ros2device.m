classdef ros2device < robotics.core.internal.mixin.Unsaveable & ...
        ros.codertarget.internal.ROS2DeviceInterface
    %ROS2DEVICE Connect to remote ROS 2 device
    %
    %   DEVICE = ROS2DEVICE(DEVICEADDRESS, USERNAME, PASSWORD) creates a
    %   ros2device object connected to the ROS 2 device at DEVICEADDRESS. Use
    %   USERNAME and PASSWORD as login credentials. The DEVICEADDRESS is an
    %   IP address such as '192.168.0.10' or a hostname such as
    %   'samplehost.foo.com'.
    %
    %   DEVICE = ROS2DEVICE creates a ros2device object connected
    %   to a ROS 2 device using saved values for DEVICEADDRESS,
    %   USERNAME and PASSWORD.
    %
    %
    %   ROS2DEVICE properties:
    %      DeviceAddress   - Hostname or IP address of the ROS device
    %      Username        - Username used to connect
    %      ROS2Folder      - Folder where ROS 2 is installed on device
    %      ROS2Workspace   - ROS 2 project folder where models are deployed on device
    %      AvailableNodes  - Nodes that are available to run on device
    %
    %   ROS2DEVICE methods:
    %      runNode       - Start ROS 2 node
    %      stopNode      - Stop ROS 2 node
    %      isNodeRunning - Determine if ROS 2 node is running
    %      system        - Execute system command on device
    %      putFile       - Copy file to device
    %      getFile       - Get file from device
    %      deleteFile    - Delete file on device
    %      dir           - List directory contents on device
    %      openShell     - Open interactive command shell to the device
    %
    %
    %   Example:
    %       % Connect to ROS 2 device
    %       device = ros2device
    %
    %       % Display all runnable nodes in the ROS 2 project folder
    %       device.AvailableNodes
    %
    %       % Run the 'ros2FeedbackControlExample' node
    %       % This model needs to be deployed to the ROS 2 device
    %       runNode(device, 'ros2FeedbackControlExample')
    %
    %       % Verify that node is running
    %       isNodeRunning(device, 'ros2FeedbackControlExample')
    %
    %       % Stop the node
    %       stopNode(device, 'ros2FeedbackControlExample')
    %
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    properties (SetAccess = private)
        %DeviceAddress - Hostname or IP address of the Remote device
        %   For example, this can be an IP address or a hostname.
        DeviceAddress
    end
    
    properties (SetAccess = private)
        %Username - Username used to connect to the device
        Username
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
        Password
        
        %Parser - Parser object for user inputs
        Parser
        
        %Diagnostic - Diagnostic helper object
        Diagnostic
    end
    
    properties
        %ROS2Folder - Folder where ROS 2 is installed
        %   This is a folder name on the ROS 2 device. By default, this is
        %   initialized from the stored settings.
        ROS2Folder

        %ROS2Workspace - ROS 2 project folder where models are deployed
        %   This is a folder name on the ROS device. By default, this
        %   is initialized from the stored settings.
        ROS2Workspace
    end    
    
    properties (Access = ?ros.slros.internal.InternalAccess)
        SystemExecutor
        NodeExecutor
    end
    
    methods
        function obj = ros2device(varargin)
            %ROSDEVICE Construct an instance of this class

            import ros.codertarget.internal.*
            initDeviceParameters(obj,varargin{:});
            
            % Create executors
            obj.SystemExecutor = createSystemExecutor(obj.DeviceAddress,...
                obj.Username,obj.Password,obj.Port);
            obj.NodeExecutor = createNodeExecutor(obj.DeviceAddress,...
                obj.SystemExecutor,'ros2');
                        
            % Automatically set ROS2Folder and ROS2Workspace variables
            if strcmpi(obj.DeviceAddress,'localhost')
                obj.ROS2Folder = obj.SystemExecutor.fullfile(matlabroot,...
                    'sys','ros2',computer('arch'),'ros2');
                obj.ROS2Workspace = pwd;
            else
                % Derive ROS2Folder and ROS2Workspace from deviceParams
                deviceParams = ros.codertarget.internal.DeviceParameters;
                obj.ROS2Folder = i_handleSpaces(deviceParams.getROS2InstallFolder);
                obj.ROS2Workspace = i_handleSpaces(deviceParams.getROS2Workspace);
            end
        end
        
        %% Getter and setter methods
        function nodeList = get.AvailableNodes(obj)
            % Return list of available nodes
            nodeList = getAvailableNodes(obj.NodeExecutor,getWorkspaceFolder(obj));
        end
        
        function set.ROS2Folder(obj,rosFolder)
            % This class is unsaveable. Hence accessing another property
            % in the set function of ROSFolder is ok
            validateattributes(rosFolder,{'char'},{'nonempty','row'},'','ROS2Folder');
            obj.ROS2Folder = rosFolder;
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
        
        %% Use node executor to implement node interface
        function runNode(obj,modelName)
            %runNode Start the node on device
            %   runNode(DEVICE, MODELNAME) starts the ROS 2 node associated
            %   with the Simulink model with name MODELNAME on the connected
            %   DEVICE. The node needs to be deployed in the project workspace
            %   folder specified in the workspace property. The node connects
            %   to the same ROS master that MATLAB is connected to and advertises
            %   its address as the property value 'DeviceAddress'.
            %
            %
            %   Example:
            %       device = ros2device
            %
            %       % Run the 'robotROSFeedbackControlExample' node
            %       % This model needs to be deployed to the ROS device
            %       runNode(device, 'robotROSFeedbackControlExample')
            %
            %       % Stop the node
            %       stopNode(device, 'robotROSFeedbackControlExample')
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
            % Send empty string for rosMasterURI and nodeHost arguments
            runNode(obj.NodeExecutor,modelName,obj.ROS2Workspace,'','');
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
            %       device = ros2device
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
            %isNodeRunning Determine if ROS 2 node is running on device
            %   ISRUNNING = isNodeRunning(DEVICE, MODELNAME) returns TRUE
            %   if the ROS 2 node associated with the Simulink model with
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
            ret = obj.ROS2Workspace;
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