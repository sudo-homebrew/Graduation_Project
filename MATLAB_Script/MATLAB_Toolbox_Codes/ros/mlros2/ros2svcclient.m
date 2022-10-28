classdef ros2svcclient < ros.ros2.internal.QOSUser & ...
        ros.internal.mixin.InternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle
%ROS2SVCCLIENT Create a ROS 2 service client
%   Use ROS2SVCCLIENT to create a ROS 2 service client object. This
%   service client uses a connection to send requests to, and receive
%   responses from, a ROS 2 service server.
%
%   CLIENT = ROS2SVCCLIENT(NODE,SVCNAME,SVCTYPE) creates a service
%   client of type SVCTYPE regardless of whether a service server offering
%   SVCNAME is available. The service client will be attached to the
%   ros2node object, NODE.
%
%   CLIENT = ROS2SVCCLIENT(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "History"     - Mode for storing requests in the queue. If the
%                      queue fills with requests waiting to be
%                      processed, then old requests are dropped to make
%                      room for new. Options are:
%                         "keeplast"       - Store up to the number of
%                                            requests set by 'Depth'.
%                         "keepall"        - Store all requests
%                                            (up to resource limits).
%      "Depth"       - Size of the request queue in number of requests.
%                      Only applies if "History" property is "keeplast".
%                      Also the number of requests persisted if
%                      Durability is "transientlocal".
%      "Reliability" - Method for ensuring request delivery.
%                      It is recommended that services use "reliable".
%                      Options are:
%                         "reliable"       - Guaranteed delivery, but
%                                            may make multiple attempts
%                                            to call.
%                         "besteffort"     - Attempt delivery once.
%      "Durability"  - Method for storing requests on the client. It is
%                      recommended that services use "volatile"
%                      durability, or else service servers that restart
%                      may receive out of date requests.
%                      Options are:
%                         "volatile"       - Requests do not persist.
%                         "transientlocal" - Recently sent requests persist.
%
%   NOTE: The "Reliability" and "Durability" quality of service settings
%   must be compatible between service servers and clients for a connection
%   to be made.
%
%   [CLIENT,REQUEST] = ROS2SVCCLIENT(___) returns a request message, REQUEST,
%   that you can use to call the CLIENT. The message will be initialized
%   with default values.
%
%
%   ROS2SVCCLIENT properties:
%      ServiceName - (Read-Only) The name of the service
%      ServiceType - (Read-Only) The type of the service
%      History     - (Read-only) Request queue mode
%      Depth       - (Read-only) Request queue size
%      Reliability - (Read-Only) Delivery guarantee of requests
%      Durability  - (Read-Only) Persistence of requests
%
%   ROS2SVCCLIENT methods:
%      ros2message       - Create a new service request message
%      isServerAvailable - Check if service server is connected
%      waitForServer     - Wait for service server to become connected
%      call              - Call the service and wait for a response
%
%
%   Example:
%      % Create a ROS 2 node
%      node = ros2node("/node_1");
%
%      % Create a service client and wait to connect to the service
%      % server (blocking). This assumes there is a service server for
%      % this service name in existence.
%      client = ROS2SVCCLIENT(node,"/camera/left/camera_info",...
%          "sensor_msgs/SetCameraInfo");
%      waitForServer(client)
%
%      % Create the service request message
%      request = ros2message(client);
%      request.camera_info.distortion_model = 'plumb_bob';
%
%      % Send the service request and wait for a response (blocking)
%      response = call(client,request);
%
%      % Send the service request and wait five seconds for a response
%      % (blocking, with timeout).
%      response = call(client,request,"Timeout",5);
%
%   See also ROS2SVCSERVER, ROS2.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %ServiceType - The type of the service
        %   This property is not dependent, since it is accessed frequently
        %   when calling a service.
        ServiceType = ''

        %ServiceName - The name of the service
        ServiceName = ''
    end

    properties (Access = private)
        %RequestType - Message type of the service request
        RequestType = ''

        %ResponseType - Message type of the service response
        ResponseType = ''

        %SyncResponse - The response received from a synchronous service call
        SyncResponse = []

        %WaitMutex - The mutex used for the synchronous service call
        WaitMutex = false

        %Currently processing request ID
        CurrentRequestId = 0
    end

    properties (Transient, Access = {?ros.internal.mixin.InternalAccess,...
                                     ?matlab.unittest.TestCase})
        %ProcessResponseCallbackHandler - Helper to handle callbacks
        ProcessResponseCallbackHandler = []

        %InternalNode - Internal representation of the node object
        %   Node required to get subscriber property information
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get subscriber property information
        ServerNodeHandle = []

        %SvcClientHandle - Designation of the service-server on the server
        %This is required to get property information.
        SvcClientHandle = []

        %ServiceInfo - includes other information for a given service
        ServiceInfo = struct.empty

        %MaxConcurrentCallbacks - Number of callbacks allowed in queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default.
        MaxConcurrentCallbacks
    end

    properties (Constant, Access = private)
        %DefaultTimeout - The default timeout for server connection
        DefaultTimeout = Inf
    end

    methods (Access = public)
        function [obj, varargout] = ros2svcclient(node, serviceName, serviceType, varargin)
        %ros2svcclient Constructor
        %   Attach a new service client to the given ROS 2 node. The "name"
        %   and "type" arguments are required and specifies the service
        %   to which this client should connect. Please see the class
        %   documentation (help ros2svcclient) for more details.

        % Parse the inputs to the constructor
            [serviceName, serviceType, varargin{:}] = ...
                convertStringsToChars(serviceName, serviceType, varargin{:});
            parser = getConstructorParser(obj);

            parse(parser, node, serviceName, serviceType, varargin{:});

            node = parser.Results.node;
            resolvedName = resolveName(node, parser.Results.serviceName);
            serviceType = parser.Results.serviceType;

            % Handle quality of service settings
            qosSettings = getQosSettings(obj, parser.Results);

            % Set object properties
            obj.ServiceType = serviceType;
            obj.RequestType = [serviceType 'Request'];
            obj.ResponseType = [serviceType 'Response'];

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode;
            obj.ServerNodeHandle = node.ServerNodeHandle;
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Service client callback
            obj.ProcessResponseCallbackHandler = ...
                ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), ...
                                             @serviceCallResponse);

            % Get service info
            obj.ServiceInfo = ros.internal.ros2.getServiceInfo(obj.ResponseType, ...
                                                               obj.ServiceType, ...
                                                               'Response');

            % Create the service client object
            createSvcClient(obj, resolvedName, qosSettings);

            % Return message structure if requested
            if nargout > 1
                varargout{1} = ros2message(obj);
            end
        end

        function delete(obj)
        %DELETE Shut down service client
        %   DELETE(CLIENT) shuts down the ROS service client object CLIENT
        %   and removes its registration from the service server.

        % Cannot tell server to remove the service client without valid
        % internal node and server handle value
            if ~isempty(obj.InternalNode) && ...
                    isvalid(obj.InternalNode) && ...
                    ~isempty(obj.SvcClientHandle)

                try
                    removeSvcClient(obj.InternalNode, ...
                                    obj.SvcClientHandle);
                catch
                    warning(message('ros:mlros2:serviceclient:ShutdownError'));
                end
            end
            obj.InternalNode = [];
        end

        function [response, status, statusText] = call(obj, varargin)
        %CALL Call the service server and receive a response
        %   RESPONSE = CALL(CLIENT) sends a default service request message
        %   and waits for a service RESPONSE. The default service request
        %   message is an empty message of type CLIENT.ServiceType.
        %
        %   RESPONSE = CALL(CLIENT,REQUEST) sends a specific service request
        %   message, REQUEST, and waits for a service RESPONSE. The type
        %   and format of the message must match the ServiceType and
        %   DataFormat of the client.
        %
        %   RESPONSE = CALL(___,Name,Value) provides additional options
        %   specified by one or more Name,Value pair arguments. You can
        %   specify several name-value pair arguments in any order as
        %   Name1,Value1,...,NameN,ValueN:
        %
        %      "Timeout" - Specifies a timeout period, in seconds.
        %                  If the service client does not receive a
        %                  service response and the timeout period
        %                  elapses, CALL displays an error message and
        %                  lets MATLAB continue running the current
        %                  program. Otherwise, the default value is Inf,
        %                  which blocks MATLAB from running the current
        %                  program until the service client receives a
        %                  service response.
        %
        %   [RESPONSE,STATUS,STATUSTEXT] = CALL(___) returns a STATUS
        %   indicating whether a RESPONSE has been received successfully.
        %   If the call fails, no error will be thrown, and STATUS will be
        %   false. An empty default RESPONSE message will be returned, and
        %   STATUSTEXT will capture information about the status.
        %   The STATUSTEXT can be one of the following:
        %
        %       'success' - The service response was successfully received.
        %       'input'   - The input to the function is invalid.
        %       'timeout' - The service response was not received within
        %                   the specified timeout.
        %       'unknown' - The service response was not received due to
        %                   an unknown error.
        %
        %   Use CALL to send a service request to a service server,
        %   and then wait to receive a response from that service server.
        %
        %   This function blocks MATLAB from running the current
        %   program until the service client receives a service response.


        % Initialize status for generic error
            status = false;
            statusText = 'unknown';

            try
                % Parse the inputs to the function
                [varargin{:}] = convertStringsToChars(varargin{:});
                parser = getCallParser(obj);
                parse(parser, varargin{:})
                requestMsg = parser.Results.requestMsg;
                callTimeout = parser.Results.Timeout;
            catch ex
                if nargout > 1
                    response = ros2message(obj.ResponseType);
                    % status already defaults to indicate error
                    statusText = 'input';
                    return
                end
                rethrow(ex)
            end

            try
                % Create default service message if not provided in call
                if isempty(requestMsg)
                    % Go straight to struct to save double-conversion
                    requestMsg = ros2message(obj);
                end

                % Blocking wait on service server response (with timeout)
                obj.WaitMutex = false;

                obj.CurrentRequestId = requestSvcServer(obj.InternalNode, ...
                                                        obj.SvcClientHandle,...
                                                        requestMsg);
            catch ex
                if nargout > 1
                    response = ros2message(obj.ResponseType);
                    % status and statusText already default to unknown error
                    return
                end
                newEx = MException(message('ros:mlros2:serviceclient:CallError', ...
                                           obj.ServiceName, obj.ServiceType));
                throw(newEx.addCause(ex));
            end

            % Wait until response received, or until timeout occurs
            try
                ros.internal.Util.getInstance.waitUntilTrue(@obj.waitState, callTimeout);
            catch
                obj.WaitMutex = true;

                %inform server.exe about timeout.
                clientRequestTimeout(obj.InternalNode, ...
                                     obj.SvcClientHandle, ...
                                     obj.CurrentRequestId, ...
                                     requestMsg);
                if nargout > 1
                    response = ros2message(obj.ResponseType);
                    % status already defaults to indicate error
                    statusText = 'timeout';
                    return
                end
                error(message('ros:mlros2:serviceclient:CallWaitTimeout', ...
                              num2str(callTimeout, '%.2f')));
            end

            % Retrieve the response if no timeout occurred
            response = obj.SyncResponse;

            % Check for error returned from back-end client
            if isfield(response, 'errName') && ...
                    isfield(response, 'errDetails')
                if nargout > 1
                    response = ros2message(obj.ResponseType);
                    % status and statusText already default to unknown error
                    return
                end
                newEx = MException(message('ros:mlros2:serviceclient:CallError', ...
                                           obj.ServiceName, ...
                                           obj.ServiceType));
                newCause = MException(strcat('ros:mlros2:serviceclient:', ...
                                             response.errName), ...
                                      response.errDetails);
                throw(newEx.addCause(newCause));
            end

            status = true;
            statusText = 'success';
        end

        function status = isServerAvailable(obj)
        % ISSERVERAVAILABLE Check whether service server is available
        %   STATUS = ISSERVERAVAILABLE(CLIENT) checks whether a service
        %   server is connected to the client and ready for communication.

            timeout = 0;    % Perform immediate check
            status = ifSvcServerConnected(obj.InternalNode, ...
                                          obj.SvcClientHandle, ...
                                          timeout);
        end

        function varargout = waitForServer(obj, varargin)
        %WAITFORSERVER Wait for service server to start and connect
        %   WAITFORSERVER(CLIENT) blocks MATLAB from running the current
        %   program until the service server is started up and available to
        %   receive request. Press Ctrl+C to abort the wait.
        %
        %   WAITFORSERVER(___,Name,Value) provides additional options
        %   specified by one or more Name,Value pair arguments. You can
        %   specify several name-value pair arguments in any order as
        %   Name1,Value1,...,NameN,ValueN:
        %
        %       "Timeout" - Specifies a timeout period, in seconds. If the
        %                   server does not start up in the timeout period,
        %                   this function displays an error message and
        %                   lets MATLAB continue running the current
        %                   program. Otherwise, the default value is Inf,
        %                   which blocks MATLAB from running the current
        %                   program until the service server is available.
        %
        %   [STATUS, STATUSTEXT] = WAITFORSERVER(____) returns a STATUS
        %   indicating whether the server is available. If the server is
        %   not available within the TIMEOUT, no error will be thrown and
        %   STATUS will be false. STATUSTEXT will capture information about
        %   the status.
        %   The STATUSTEXT can be one of the following:
        %
        %       'success' - The server is available.
        %       'input'   - The input to the function is invalid.
        %       'timeout' - The server did not become available before the
        %                   timeout period expired.

        % Initialize status as false
            status = false;

            try
                % Parse name-value pairs
                parser = inputParser;
                addParameter(parser, 'Timeout', obj.DefaultTimeout, ...
                             @(x) validateattributes(x, {'numeric'}, ...
                                                     {'scalar', 'real', 'positive'}, 'waitForServer', 'timeout'));
                parse(parser, varargin{:});
                timeout = parser.Results.Timeout;
            catch ex
                if nargout > 0
                    statusText = 'input';
                    varargout = {status, statusText};
                    return
                end
                rethrow(ex)
            end

            % Wait until service is available (or timeout occurs)
            try
                ros.internal.Util.getInstance.waitUntilTrue(@() isServerAvailable(obj), ...
                                                            timeout);
            catch
                if nargout > 0
                    statusText = 'timeout';
                    varargout = {status, statusText};
                    return
                end
                error(message('ros:mlros2:serviceclient:ConnectWaitTimeout', ...
                              obj.ServiceName, num2str(timeout, '%.2f')));
            end

            status = true;
            statusText = 'success';

            % Only return output if requested
            if nargout > 0
                varargout = {status, statusText};
            end
        end

        function msg = ros2message(obj)
        % ROS2MESSAGE Create a new service request message
        %   REQUEST = ROS2MESSAGE(CLIENT) creates and returns an empty message REQUEST.
        %   The message type of REQUEST is determined by the service that this
        %   CLIENT is connected to. The message is the default request that
        %   you can use to call a service.
        %
        %   Example:
        %      % Create a node and service client
        %      node = ros2node("/sensors");
        %      client = ros2svcclient(node,"/camera/left/camera_info","sensor_msgs/SetCameraInfo");
        %
        %      % Create request message
        %      request = ROS2MESSAGE(client);
        %
        %   See also CALL.

            msg = ros2message(obj.RequestType);
        end
    end

    methods (Access = private)
        function createSvcClient(obj, serviceName, qosSettings)
        %creates service client on ROS network
            callbackFcn = obj.ProcessResponseCallbackHandler.CallbackName;
            dllPathsRequest = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Request'],'ros2');
            dllPathsResponse = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Response'],'ros2');
            dllPaths = unique([dllPathsRequest dllPathsResponse]);

            try
                returnCall = addSvcClient(obj.InternalNode, ...
                                          obj.ServerNodeHandle, ...
                                          serviceName, ...
                                          obj.ServiceInfo.path, ...
                                          obj.ServiceInfo.cppFactoryClass, ...
                                          callbackFcn, ...
                                          qosSettings, ...
                                          dllPaths);

                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlros2:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle) || ...
                        ~isfield(returnCall, 'svcName') || ...
                        isempty(returnCall.svcName)
                    error(message('ros:mlros2:node:InvalidReturnCallHandleError'))
                end
                obj.SvcClientHandle = returnCall.handle;
                obj.ServiceName = returnCall.svcName;

                % Initialize callback to process requests.
                initSvcClientCallback(obj.InternalNode, ...
                                      returnCall.handle, ...
                                      obj.ProcessResponseCallbackHandler, ...
                                      obj.MaxConcurrentCallbacks);
                % No need to check reply - should error on failure
            catch ex
                newEx = MException(message('ros:mlros2:serviceclient:CreateError', ...
                                           obj.ServiceName, obj.ServiceType));
                throw(newEx.addCause(ex));
            end
        end

        function value = waitState(obj)
        %waitState Return state of mutex
        %   This function is repeatedly evaluated while waiting for the service
        %   server response.

            value = obj.WaitMutex;
        end
    end

    methods (Access = ?ros.ros2.internal.SvcClientCallbackHandler)
        function serviceCallResponse(obj, response, clientInfo, requestInfo)
        %serviceCallResponse Receive response for service call
        %   This function is called if a service call response is
        %   received. The response will be received
        %   asynchronously. Since the call behavior of this client is
        %   synchronous, the WaitMutex variable is used to block client
        %   execution until a response is received through this function.
        %
        %   ROS 2 has no way of informing the client of a failure on the
        %   side of the service server, so any response should be
        %   indicative of success.

            hClient = clientInfo.handle;
            requestId = requestInfo.requestId;

            % Check if the mutex state is waiting and the callback is
            % correct else ignore the callback
            if obj.waitState || ...
                    ~isequal(obj.SvcClientHandle,hClient) || ...
                    ~isequal(obj.CurrentRequestId,requestId)
                return
            end
            obj.SyncResponse = response;
            obj.WaitMutex = true;
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function parser = getConstructorParser(obj)
        %getConstructorParser Set up parser for constructor inputs

        % Set up ordered inputs
        % node and service name are always needed, type is optional
            parser = inputParser;
            addRequired(parser, 'node',  @(x) ...
                        validateattributes(x, {'ros2node'}, ...
                                           {'scalar'}, ...
                                           'ros2svcclient', 'node'));
            addRequired(parser, 'serviceName',  @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'scalartext', 'nonempty'}, ...
                                           'ros2svcclient', 'serviceName'));
            addRequired(parser,'serviceType', @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'scalartext', 'nonempty'}, ...
                                           'ros2svcclient','serviceType'));
            parser = addQOSToParser(obj, parser, 'ros2svcclient');
        end

        function parser = getCallParser(obj)
        %getCallParser Set up parser for call method inputs

            parser = inputParser;
            % Keep parser for requestMsg general to both acceptable formats
            % to allow for custom error message due to later failure
            addOptional(parser, 'requestMsg', [], ...
                        @(x) validateattributes(x, {'struct'}, ...
                                                {'scalar'}, ...
                                                'call', 'request'));
            addParameter(parser, 'Timeout', obj.DefaultTimeout, ...
                         @(x) validateattributes(x, {'numeric'}, ...
                                                 {'scalar','nonempty','positive','nonnan'}, ...
                                                 'call', 'Timeout'));
        end
    end

    methods (Access = protected)
        function svcClientInfo = getServerInfo(obj)
        %getServerInfo Get service client properties from node server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlros2:serviceclient:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.SvcClientHandle)
                error(message('ros:mlros2:serviceclient:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlros2:serviceclient:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            svcClientHandles = [nodeInfo.svcclients.handle];
            whichServer = obj.SvcClientHandle == svcClientHandles;
            if ~any(whichServer)
                % Must be the wrong handle(s) if this service client exists
                error(message('ros:mlros2:serviceclient:InvalidServerHandleError'))
            elseif nnz(whichServer) > 1
                % Duplicate service client handles found, error on node side
                error(message('ros:mlros2:serviceclient:DuplicateSvcClientHandlesError'))
            end
            svcClientInfo = nodeInfo.svcclients(whichServer);
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ros2svcclient';
        end
    end
end
