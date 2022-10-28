classdef ServiceClient < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
%ServiceClient Create a ROS service client
%   Use ServiceClient to create a ROS service client object. This service client
%   uses a persistent connection to send requests to, and receive
%   responses from, a ROS service server. The connection persists until
%   the service client is deleted or the service server becomes
%   unavailable.
%
%   CLIENT = ros.ServiceClient(NODE,SVCNAME) creates a service client that
%   connects to, and gets its service type from, a service server.
%   This command syntax blocks the current MATLAB program from running
%   until it can connect to the service server. The service client will
%   be attached to the ros.Node object, NODE. SVCNAME is a string scalar.
%
%   CLIENT = ros.ServiceClient(NODE,SVCNAME,SVCTYPE) creates a service
%   client of type SVCTYPE regardless of whether a service server offering
%   SVCNAME is available. If SVCNAME already exists in the network,
%   SVCTYPE must match the service type of SVCNAME. The service client
%   will be attached to the ros.Node object, NODE. SVCNAME is a string scalar.
%
%   CLIENT = ros.ServiceClient(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of ROS message to be used by
%                     the service client, and returned from rosmessage.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%      "Timeout"    - Specifies a timeout period, in seconds. If the
%                     service client does not connect to the service server
%                     and the timeout period elapses, ros.ServiceClient displays
%                     an error message and lets MATLAB continue running
%                     the current program. Otherwise, the default value is Inf,
%                     which blocks MATLAB from running the current
%                     program until the service client receives a
%                     service response.
%
%
%   ServiceClient properties:
%      ServiceName  - (Read-Only) The name of the service
%      ServiceType  - (Read-Only) The type of the service
%      DataFormat   - (Read-Only) Service message format required for use
%
%   ServiceClient methods:
%      rosmessage   - Create a new service request message
%      call         - Call the service and receive a response
%
%
%   Example:
%
%      % Create a service client and wait to connect to the service
%      % server (blocking). This assumes that there is an existing node object
%      % Use struct message format for better performance
%      client = ros.ServiceClient(node,"/gazebo/get_model_state",...
%          "DataFormat","struct");
%
%      % Create the service request message
%      request = rosmessage(client);
%      request.ModelName = 'SomeModel';
%
%      % Send the service request and wait for a response (blocking)
%      response = call(client,request);
%
%      % Send the service request and wait five seconds for a response
%      % (blocking, with timeout).
%      response = call(client,request,"Timeout",5);
%
%      % Create and call with another service client that uses message objects
%      % Wait three seconds to connect to the service server (blocking, with timeout)
%      client2 = ros.ServiceClient(node,"/gazebo/get_model_state",...
%          "Timeout",3,"DataFormat","object");
%      requestObj = rosmessage(client2)
%      responseObj = call(client2,requestObj);
%
%   See also ROSSVCCLIENT, ros.ServiceServer, ROSSERVICE.

%   Copyright 2014-2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %ServiceType - The type of the service
        %   This property is not dependent, since it is accessed frequently
        %   when calling a service.
        ServiceType = ''

        %ServiceName - The name of the service
        ServiceName = ''
    end

    properties (Access = private)
        %ServiceURI - The URI at which the service server can be reached
        ServiceURI = ''

        %MasterURI - The URI of the ROS master
        MasterURI

        %RequestType - Message type of the service request
        RequestType = ''

        %ResponseType - Message type of the service response
        ResponseType = ''

        %SyncResponse - The response received from a synchronous service call
        SyncResponse = []

        %WaitMutex - The mutex used for the synchronous service call
        WaitMutex = false

        %Currently processing request id.
        CurrentRequestId = 0
    end
    properties (Transient, Access = {?ros.internal.mixin.ROSInternalAccess,...
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
        %DefaultTimeout - The default timeout for constructor
        DefaultTimeout = Inf
    end

    methods (Access = public)
        function obj = ServiceClient(node, serviceName, varargin)
        %ServiceClient Constructor
        %   Attach a new service client to the given ROS node. The "name"
        %   argument is required and specifies the service to which this
        %   client should connect. Please see the class documentation
        %   (help ros.ServiceClient) for more details.

        % If no node specified, use the global node.
            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Parse the inputs to the constructor
            [serviceName, varargin{:}] = convertStringsToChars(serviceName, varargin{:});
            [ordinalParser, paramParser] = getConstructorParser(obj);
            nvPairsStart = ros.internal.Parsing.findNameValueIndex(...
                varargin, paramParser.Parameters);
            if isempty(nvPairsStart)
                % Additional arguments past type assume to be parameters
                nvPairsStart = min(2,numel(varargin)+1);
            end

            parse(ordinalParser, node, serviceName, varargin{1:nvPairsStart-1});
            parse(paramParser, varargin{nvPairsStart:end});

            node = ordinalParser.Results.node;
            serviceName = ordinalParser.Results.serviceName;
            serviceType = ordinalParser.Results.serviceType;
            connectTimeout = paramParser.Results.Timeout;

            masterURI = node.MasterURI;

            % Make sure that message type agree if service is already
            % available
            % Retrieve type of ROS service if not specified
            if isempty(serviceType)
                % Wait until service is available (or timeout occurs)
                try
                    ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                                 ros.internal.NetworkIntrospection.isServiceAvailable(...
                        serviceName, masterURI), ...
                                                                 connectTimeout );
                catch
                    error(message('ros:mlros:serviceclient:ConnectWaitTimeout', ...
                                  serviceName, num2str(connectTimeout, '%.2f')));
                end
                serviceType = ros.internal.NetworkIntrospection.getServiceType(serviceName, masterURI);
            else
                % check once to make sure the specified service type
                % matches what's already on the network
                masterType = ros.internal.NetworkIntrospection.getServiceType(serviceName, masterURI);
                if ~isempty(masterType) && ...
                        ~strcmp(masterType,serviceType)
                    error(message('ros:mlros:serviceclient:ServiceTypeNoMatch', ...
                                  serviceName,masterType,masterType,serviceType));
                end
            end

            % Retrieve URI
            serviceURI = ros.internal.NetworkIntrospection.getServiceURI(serviceName, masterURI);

            % Set object properties
            obj.ServiceType = serviceType;
            obj.ServiceURI = serviceURI;
            obj.MasterURI = masterURI;
            obj.RequestType = [serviceType 'Request'];
            obj.ResponseType = [serviceType 'Response'];
            setDataFormat(obj, paramParser.Results.DataFormat)

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode; %ros.internal.Node
            obj.ServerNodeHandle = node.ServerNodeHandle; %node handle for server
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Service client callback
            obj.ProcessResponseCallbackHandler = ...
                ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), ...
                                             @serviceCallResponse);

            % Get service info
            obj.ServiceInfo = ros.internal.ros.getServiceInfo(obj.ResponseType, ...
                                                              obj.ServiceType, ...
                                                              'Response');

            % Store the service client handle in node object
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);

            % Create the service client object
            createSvcClient(obj,serviceName);
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
                    warning(message('ros:mlros:serviceclient:ShutdownError'));
                end
            end
            obj.InternalNode = [];
        end

        function [response, status, statusText] = call(obj, varargin)
        %CALL Call the service server and receive a response
        %   RESPONSE = CALL(CLIENT) sends a default service request message
        %   and waits for a service RESPONSE. The default service
        %   request message is an empty message of type CLIENT.ServiceType.
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
        %   The STATUSTEXT can be one of the following string:
        %
        %       'success' - The service response was successfully received.
        %       'input'   - The input to the function is invalid.
        %       'timeout' - The service response was not received within
        %                   the specified timeout.
        %       'unknown' - The service response was not received due to
        %                   unknown errors.
        %
        %   Use CALL to send a service request to a service server,
        %   and then waits to receive a response from that service server.
        %
        %   This function blocks MATLAB from running the current
        %   program until the service client receives a service response.


        % Initialize status as failed for unknown
            status = false;
            statusText = 'unknown';
            % Initialize response as default response message
            response = rosmessage(obj.ResponseType, 'DataFormat', obj.DataFormat);

            try
                % Parse the inputs to the function
                [varargin{:}] = convertStringsToChars(varargin{:});
                parser = getCallParser(obj);
                parse(parser, varargin{:})
                requestMsg = parser.Results.requestMsg;
                callTimeout = parser.Results.timeout;
            catch ex
                if nargout > 1
                    % status already defaults to indicate error
                    statusText = 'input';
                    return
                end
                rethrow(ex)
            end

            persistent gNextId;

            try
                % Create default service message if not provided in call
                if isempty(requestMsg)
                    % Go straight to struct to save double-conversion
                    requestMsgStruct = rosmessage(obj.RequestType, 'DataFormat', 'struct');
                elseif obj.UseObjectMsg
                    requestMsgStruct = toROSStruct(requestMsg);
                else
                    requestMsgStruct = requestMsg;
                end

                % Blocking wait on service server response (with timeout)
                obj.WaitMutex = false;

                % Set the request id
                if isempty(gNextId)
                    gNextId = int64(1);
                else
                    gNextId = gNextId + 1;
                end
                obj.CurrentRequestId = gNextId;
                requestMsgStruct.requestId = gNextId;

                requestSvcServer(obj.InternalNode, ...
                                 obj.SvcClientHandle,...
                                 requestMsgStruct);
            catch ex
                if ~isempty(requestMsg)
                    validateInputMessage(obj, requestMsg, obj.RequestType, 'ServiceClient', 'call')
                end
                rethrow(ex)
            end

            % Wait until response received, or until timeout occurs
            try
                ros.internal.Util.getInstance.waitUntilTrue( @obj.waitState, callTimeout );
            catch
                obj.WaitMutex = true;

                %inform server.exe about timeout.
                clientRequestTimeout(obj.InternalNode, ...
                                     obj.SvcClientHandle,...
                                     obj.CurrentRequestId,...
                                     requestMsgStruct);
                if nargout<2
                    error(message('ros:mlros:serviceclient:CallWaitTimeout', ...
                                  num2str(callTimeout, '%.2f')));
                else
                    statusText = 'timeout';
                    return
                end
            end

            % If the response is a string, a failure occurred
            if ischar(obj.SyncResponse)
                if nargout<2
                    error(message('ros:mlros:serviceclient:CallFailure', ...
                                  obj.SyncResponse));
                else
                    return
                end
            end

            % Retrieve the response if no timeout occurred
            response = obj.SyncResponse;
            if obj.UseObjectMsg
                response = feval(obj.ServiceInfo.msgClassGen, response);
            end

            status = true;
            statusText = 'success';
        end

        function status = isServerAvailable(obj)
        % ISSERVERAVAILABLE Check whether service server is available
        %   STATUS = ISSERVERAVAILABLE(CLIENT) checks whether a service
        %   server with the same service name as this service client
        %   is available.

            status = ros.internal.NetworkIntrospection.isServiceAvailable( ...
                obj.ServiceName, obj.MasterURI);
        end

        function varargout = waitForServer(obj, varargin)
        %WAITFORSERVER Wait for service server to start
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
                addParameter(parser,'Timeout', obj.DefaultTimeout, ...
                             @(x) validateattributes(x,{'numeric'}, ...
                                                     {'scalar', 'real', 'positive'},'waitForServer','timeout'));
                parse(parser,varargin{:});
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
                ros.internal.Util.getInstance.waitUntilTrue( @() ...
                                                             ros.internal.NetworkIntrospection.isServiceAvailable(...
                    obj.ServiceName, obj.MasterURI), ...
                                                             timeout);
            catch
                if nargout > 0
                    statusText ='timeout';
                    varargout = {status, statusText};
                    return
                end
                error(message('ros:mlros:serviceclient:ConnectWaitTimeout', ...
                              obj.ServiceName, num2str(timeout, '%.2f')));
            end

            status = true;
            statusText = 'success';

            % Only return output if requested
            if nargout > 0
                varargout = {status, statusText};
            end
        end

        function msg = rosmessage(obj, varargin)
        % ROSMESSAGE Create a new service request message
        %   MSG = ROSMESSAGE(CLIENT) creates and returns an empty message MSG.
        %   The message type of MSG is determined by the service that this
        %   CLIENT is connected to. The format of MSG is determined by the
        %   DataFormat of the service client. The message is the default
        %   request that you can use to call a service.
        %
        %   Example:
        %      % Create a service client and request message
        %      client1 = ros.ServiceClient(node,"/gazebo/get_model_state");
        %      msgObj = ROSMESSAGE(client1);
        %
        %      % Improve performance by using struct messages
        %      client2 = ros.ServiceClient(node,"/gazebo/get_model_state",...
        %          "DataFormat","struct");
        %      msgStruct = ROSMESSAGE(client2)
        %
        %   See also CALL.

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.RequestType, 'DataFormat', obj.DataFormat);
        end
    end

    methods (Access = private)
        function createSvcClient(obj,varargin)
        %creates service client on ROS network

            callbackFcn = obj.ProcessResponseCallbackHandler.CallbackName;

            % As both ROS1 and ROS2 uses same MCOS API and ROS2-Services
            % has QOS, while using the same API for ROS1, a dummy value is
            % being sent as QOS to backend which is not going o be used
            % anywhere.
            dummyQOS = struct();

            dllPathsRequest = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Request'],'ros');
            dllPathsResponse = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Response'],'ros');
            dllPaths = [dllPathsRequest dllPathsResponse];
            try
                returnCall = addSvcClient(obj.InternalNode, ...
                                          obj.ServerNodeHandle, ...
                                          varargin{1}, ...
                                          obj.ServiceInfo.path, ...
                                          obj.ServiceInfo.cppFactoryClass, ...
                                          callbackFcn, ...
                                          dummyQOS, ...
                                          dllPaths);

                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle) || ...
                        ~isfield(returnCall, 'svcName') || ...
                        isempty(returnCall.svcName)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
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
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:serviceclient:CreateError', obj.ServiceName));
                throw(newEx.addCustomCause(ex));
            end
        end

        function value = waitState(obj)
        %waitState Return state of mutex
        %   This function is repeatedly evaluated while waiting for the service
        %   server response.

            value = obj.WaitMutex;
        end
    end

    methods (Access = ?ros.internal.SvcClientCallbackHandler)
        function serviceCallResponse(obj, response, clientInfo, requestInfo)
        %serviceCallResponse Receive response for service call
        %   This function is called if a service call response is
        %   received. The response will be received
        %   asynchronously. Since the call behavior of this client is
        %   synchronous, the WaitMutex variable is used to block client
        %   execution until a response is received through this function.
        %
        %   It handles both success and failure cases. If response
        %   received successfully, sets the response to
        %   obj.SyncResponse. If failure occurs, the response will
        %   have a property called serverCallbackError. In that case,
        %   it gets the error message and sets that to
        %   obj.SyncResponse.

            hClient = clientInfo.handle;
            requestId = requestInfo.requestId;

            % Check if the mutex state is waiting and the callback is
            % correct else ignore the callback
            if obj.waitState || ...
                    ~isequal(obj.SvcClientHandle,hClient) || ...
                    ~isequal(obj.CurrentRequestId,requestId)
                return
            end

            if isfield(response,'serverCallbackError')
                obj.SyncResponse = response.serverCallbackError;
            else
                obj.SyncResponse = response;
            end
            obj.WaitMutex = true;
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function [ordinalParser, paramParser] = getConstructorParser(obj)
        %getConstructorParser Set up parser for constructor inputs

        % Set up ordered inputs
        % node and service name are always needed, type is optional
            ordinalParser = inputParser;
            addRequired(ordinalParser, 'node',  @(x) ...
                        validateattributes(x, {'ros.Node'}, ...
                                           {'scalar', 'nonempty'}, ...
                                           'ServiceClient', 'node'));
            addRequired(ordinalParser, 'serviceName',  @(x) ...
                        validateattributes(x, {'char'}, {'nonempty'}, ...
                                           'ServiceClient', 'serviceName'));
            addOptional(ordinalParser,'serviceType', '', @(x) ...
                        validateattributes(x, {'char'}, {'nonempty'}, ...
                                           'ServiceClient','serviceType'));

            % Set up name-value pairs
            paramParser = inputParser;
            addParameter(paramParser, 'Timeout', obj.DefaultTimeout, ...
                         @(x) validateattributes(x, {'numeric'}, ...
                                                 {'scalar','nonempty','positive','nonnan'}, ...
                                                 'ServiceClient', 'Timeout'));
            addDataFormatToParser(obj, paramParser, 'ServiceClient')
        end

        function parser = getCallParser(obj)
        %getCallParser Set up parser for call method inputs

            parser = inputParser;
            % Keep parser for requestMsg general to both acceptable formats
            % to allow for custom error message due to later failure
            addOptional(parser, 'requestMsg', [], ...
                        @(x) validateattributes(x, {'ros.Message', 'struct'}, ...
                                                {'scalar'}, ...
                                                'call', 'request'));
            addParameter(parser, 'timeout', obj.DefaultTimeout, ...
                         @(x) validateattributes(x, {'numeric'}, ...
                                                 {'scalar','nonempty','positive','nonnan'}, ...
                                                 'call', 'timeout'));
        end
    end

    methods(Access = private, Static)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ServiceClient';
        end
    end
end
