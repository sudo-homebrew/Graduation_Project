classdef ros2svcserver < ros.ros2.internal.QOSUser & ...
        ros.internal.mixin.InternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle
%ROS2SVCSERVER Create a ROS 2 service server object
%   Use ROS2SVCSERVER to create a ROS 2 service server that can receive
%   requests from, and send responses to, a ROS 2 service client.
%
%   When you create the service server, it registers itself with the
%   ROS 2 network. To get a list of services that are available on the
%   current ROS 2 network, or to get more information about any particular
%   service, use the ros2 function.
%
%   The service is defined by a type and a pair of messages: one for the
%   request and one for the response. The service server will receive a
%   request, construct an appropriate response, and return it to the
%   client. The behavior of the service server is inherently asynchronous,
%   as it only becomes active when a service client connects and issues
%   a call.
%
%   SERVER = ROS2SVCSERVER(NODE,SVCNAME,SVCTYPE,CB) creates and returns a
%   service server object. The service will be available in the ROS network
%   through its name SVCNAME and has the type SVCTYPE. The service server
%   will be attached to the ros2node object NODE. SVCNAME and SVCTYPE are
%   string scalars. It also specifies the function handle callback, CB,
%   that constructs a response when the server receives a request.
%   CB can be a single function handle or a cell array. The first element
%   of the cell array must be a function handle, or a string containing
%   the name of a function. The remaining elements of the cell array can
%   be arbitrary user data that is passed to the callback function.
%
%   SERVER = ROS2SVCSERVER(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments.
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
%      "Reliability" - Requirement on request and response delivery.
%                      It is recommended that services use "reliable".
%                      Options are:
%                         "reliable"       - Guaranteed delivery, but
%                                            may make multiple attempts.
%                         "besteffort"     - Attempt delivery once.
%      "Durability"  - Requirement on request persistence on the client.
%                      It is recommended that services use "volatile"
%                      durability, or else service servers that restart
%                      may receive out of date requests.
%                      Options are:
%                         "volatile"       - Requests do not persist.
%                         "transientlocal" - Recently sent requests must persist.
%
%   NOTE: The "Reliability" and "Durability" quality of service settings
%   must be compatible between service servers and clients for a connection
%   to be made.
%
%   The service server callback function requires at least two input
%   arguments and one output. The first argument, REQUEST, is the
%   request message sent by the service client. The second argument is
%   the default response message, DEFAULTRESPONSE.  Use the
%   DEFAULTRESPONSE as a starting point for constructing the function
%   output RESPONSE, which will be sent back to the service client
%   after the function returns.
%
%      function RESPONSE = serviceCallback(REQUEST,DEFAULTRESPONSE)
%          RESPONSE = DEFAULTRESPONSE;
%          % Build the response message here
%      end
%
%   While setting the callback, to construct a callback that accepts
%   additional parameters, use a cell array that includes both the
%   function handle and the parameters.
%
%
%   ServiceServer properties:
%      ServiceName   - (Read-Only) The name of the service
%      ServiceType   - (Read-Only) The type of the service
%      NewRequestFcn - Callback property for service request callbacks
%      History       - (Read-only) Request queue mode
%      Depth         - (Read-only) Request queue size
%      Reliability   - (Read-Only) Delivery guarantee of communication
%      Durability    - (Read-Only) Persistence requirement on requests
%
%   ServiceServer methods:
%      ros2message   - Create a new service response message
%
%
%   Example:
%      % Create a ROS 2 node
%      node = ros2node("/node_1");
%
%      % Create a service server and assign a callback
%      server = ROS2SVCSERVER(node,"/camera/left/camera_info",...
%          "sensor_msgs/SetCameraInfo",@serverCallback);
%
%   See also ROS2SVCCLIENT, ROS2.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %ServiceType - The type of the service
        %   This property is not dependent, since it is accessed frequently
        %   when service requests are received.
        ServiceType = ''

        %ServiceName - The name of the service
        ServiceName = ''
    end

    properties
        %NewRequestFcn - Callback property for service request callbacks
        NewRequestFcn = function_handle.empty;
    end

    properties (Access = private)
        %RequestType - Message type of the service request
        RequestType = ''

        %ResponseType - Message type of the service response
        ResponseType = ''
    end

    properties (Transient, Access = {?ros.internal.mixin.InternalAccess,...
                                     ?matlab.unittest.TestCase})
        %ProcessRequestCallbackHandler - Helper to handle callbacks
        ProcessRequestCallbackHandler = []

        %ActualNewRequestFcn - Function to call when request received
        %   Function handle or string
        %   User-provided additional arguments held separately
        ActualNewRequestFcn = function_handle.empty

        %NewRequestCallbackArgs - User-provided additional arguments
        %   Cell array
        NewRequestCallbackArgs = {}

        %InternalNode - Internal representation of the node object
        %   Node required to get subscriber property information
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get subscriber property information
        ServerNodeHandle = []

        %ServiceServerHandle - Designation of the service-server on the server
        %This is required to get property information.
        SvcServerHandle = []

        %ServiceInfo - includes other information for a given service
        ServiceInfo = struct.empty

        %MaxConcurrentCallbacks - Number of callbacks allowed in queue
        %   This is separate from the History/Depth Quality of Service
        %   combination, as that limits the ROS 2 processing queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default.
        MaxConcurrentCallbacks
    end

    methods
        function obj = ros2svcserver(node, serviceName, serviceType, cb, varargin)
        %ServiceServer Constructor
        %   Attach a new service server to the given ROS 2 node. The
        %   "name", "type", and "cb" arguments are required and specify the
        %   advertised service name and type, and new request callback.
        %   Please see the class documentation (help ros2svcserver) for
        %   more details.

        % Parse the inputs to the constructor
            [serviceName, serviceType, cb, varargin{:}] = ...
                convertStringsToChars(serviceName, serviceType, cb, varargin{:});
            parser = getParser(obj);
            parse(parser, node, serviceName, serviceType, cb, varargin{:})
            node = parser.Results.node;
            resolvedName = resolveName(node, parser.Results.serviceName);
            serviceType = parser.Results.serviceType;
            requestFcn = parser.Results.requestFcn;

            % Handle quality of service settings
            qosSettings = getQosSettings(obj, parser.Results);

            % Set some object properties
            obj.ServiceType = serviceType;
            obj.RequestType = [serviceType 'Request'];
            obj.ResponseType = [serviceType 'Response'];

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode;
            obj.ServerNodeHandle = node.ServerNodeHandle;
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Get service info
            obj.ServiceInfo = ros.internal.ros2.getServiceInfo(obj.RequestType, ...
                                                               obj.ServiceType, ...
                                                               'Request');

            % Initialize callback handler object
            obj.ProcessRequestCallbackHandler = ...
                ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), ...
                                             @processRequest);

            % Set the callback
            try
                obj.NewRequestFcn = requestFcn;
            catch ex
                newEx = MException(message('ros:mlros2:serviceserver:CreateError', ...
                                           resolvedName, serviceType));
                throw(newEx.addCause(ex));
            end

            % Create the service server object
            createSvcServer(obj, resolvedName, qosSettings);
        end

        function delete(obj)
        %DELETE Shut down service server
        %   DELETE(SERVER) shuts down the ROS 2 service server object SERVER
        %   and removes it from the network

        % Cannot tell server to remove the service client without valid
        % internal node and server handle value
            if ~isempty(obj.InternalNode) && ...
                    isvalid(obj.InternalNode) && ...
                    ~isempty(obj.SvcServerHandle)

                try
                    removeSvcServer(obj.InternalNode, ...
                                    obj.SvcServerHandle);
                catch
                    warning(message('ros:mlros2:serviceserver:ShutdownError'));
                end
            end
            obj.InternalNode = [];
        end

        function msg = ros2message(obj)
        % ROS2MESSAGE Create a new service response message
        %   RESPONSE = ROS2MESSAGE(SERVER) creates and returns an empty message RESPONSE.
        %   The message type of RESPONSE is determined by the service type. The
        %   message is the default response that this server can use to reply
        %   to client requests.
        %
        %   Example:
        %      % Create a node and service server
        %      node = ros2node("/sensors");
        %      server = ros2svcserver(node,"/camera/left/camera_info","sensor_msgs/SetCameraInfo");
        %
        %      % Create response message
        %      response = ROS2MESSAGE(server);

            msg = ros2message(obj.ResponseType);
        end

        function set.NewRequestFcn(obj, requestFcn)
        %set.NewRequestFcn Set the callback for new service requests
        %
        %   server.NewRequestFcn = CB sets the callback function that should
        %   be invoked when a new service request is received. Here, CB is
        %   either a scalar function handle or a cell array. You can
        %   pass additional parameters to the callback function by including
        %   both the function handle and the parameters as elements
        %   of a cell array and assign it to CB.
        %
        %   Each callback function must have the
        %   following signature (see the help for ros2svcserver for
        %   a more detailed explanation):
        %
        %      function RESPONSE = serviceCallback(REQUEST, DEFAULTRESPONSE)
        %          RESPONSE = DEFAULTRESPONSE;
        %          % Build the response message here
        %      end

            if ~isempty(obj.ProcessRequestCallbackHandler) %#ok<MCSUP>
                                                           % Make sure this is a valid function specifier
                [funcHandle, userData] = ...
                    ros.internal.Parsing.validateFunctionHandle(requestFcn);

                % Set properties used when message is received
                obj.ActualNewRequestFcn = funcHandle; %#ok<MCSUP>
                obj.NewRequestCallbackArgs = userData; %#ok<MCSUP>
            else
                obj.ActualNewRequestFcn = function_handle.empty; %#ok<MCSUP>
                obj.NewRequestCallbackArgs = {}; %#ok<MCSUP>
            end
            obj.NewRequestFcn = requestFcn;
        end
    end


    methods (Access = ?ros.internal.mixin.InternalAccess)
        function processRequest(obj, requestMsg, varargin)
        % processRequest takes action based on new request from client

        % Call the callback function
            try
                % Default response
                response = ros2message(obj);

                % Evaluate the response
                response = feval(obj.ActualNewRequestFcn, ...
                                 requestMsg, ...
                                 response, ...
                                 obj.NewRequestCallbackArgs{:});
            catch ex
                % Send error message back to back-end if something goes
                % wrong due to exception or user mistake in callback

                warning(message('ros:mlros2:serviceserver:UserCallbackError', ex.message))
                errorResponse.message = ex.message;
                sendBackErrorResponse(obj, errorResponse);
            end

            % Send the response back over the network
            try
                sendBackResponse(obj, response);
            catch ex
                % Try to send error message back to back-end if something
                % goes wrong with the server or network

                warning(message('ros:mlros2:serviceserver:SendResponseError', ex.message))
                errorResponse.message = ex.message;
                sendBackErrorResponse(obj, errorResponse);
            end
        end
    end

    methods (Access = private)
        function createSvcServer(obj, serviceName, qosSettings)
        %creates service-server on ROS 2 network
            callbackFcn = obj.ProcessRequestCallbackHandler.CallbackName;
            dllPathsRequest = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Request'],'ros2');
            dllPathsResponse = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Response'],'ros2');
            dllPaths = unique([dllPathsRequest dllPathsResponse]);

            try
                returnCall = addSvcServer(obj.InternalNode, ...
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
                obj.SvcServerHandle = returnCall.handle;
                obj.ServiceName = returnCall.svcName;

                % Initialize callback to process requests.
                initSvcServerCallback(obj.InternalNode, ...
                                      returnCall.handle, ...
                                      obj.ProcessRequestCallbackHandler, ...
                                      obj.MaxConcurrentCallbacks);
                % No need to check reply - should error on failure
            catch ex
                newEx = MException(message('ros:mlros2:serviceserver:CreateError', ...
                                           obj.ServiceName, obj.ServiceType));
                throw(newEx.addCause(ex));
            end
        end

        function sendBackResponse(obj,response)
        % Send back the response to server

            responseFromMLtoSvcServer(obj.InternalNode, ...
                                      obj.SvcServerHandle, ...
                                      response);
        end

        function sendBackErrorResponse(obj,response)
        % This unlocks the mutex lock in back-end server because server is not
        % going to get the response back from the callback in case of exception.
        % No error response is sent to the client.

            responseFromMLonCallbackError(obj.InternalNode, ...
                                          obj.SvcServerHandle, ...
                                          response);
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function parser = getParser(obj)
        %getParser Set up parser for constructor inputs

            parser = inputParser;

            % Node, service name, and service type are required inputs
            addRequired(parser, 'node',  @(x) ...
                        validateattributes(x, {'ros2node'}, ...
                                           {'scalar'}, ...
                                           'ros2svcserver', 'node'));
            addRequired(parser, 'serviceName',  @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'scalartext', 'nonempty'}, ...
                                           'ros2svcserver', 'svcName'));
            addRequired(parser, 'serviceType',  @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'scalartext', 'nonempty'}, ...
                                           'ros2svcserver', 'svcType'));
            addRequired(parser, 'requestFcn', ...
                        @(x) validateattributes(x, ...
                                                {'function_handle', 'cell'}, ...
                                                {'nonempty'}, ...
                                                'ros2svcserver', 'cb'));
            parser = addQOSToParser(obj, parser, 'ros2svcserver');
        end
    end

    methods (Access = protected)
        function svcServerInfo = getServerInfo(obj)
        %getServerInfo Get service server properties from node server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlros2:serviceserver:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.SvcServerHandle)
                error(message('ros:mlros2:serviceserver:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlros2:serviceserver:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            svcServerHandles = [nodeInfo.svcservers.handle];
            whichServer = obj.SvcServerHandle == svcServerHandles;
            if ~any(whichServer)
                % Must be the wrong handle(s) if this service server exists
                error(message('ros:mlros2:serviceserver:InvalidServerHandleError'))
            elseif nnz(whichServer) > 1
                % Duplicate service server handles found, error on node side
                error(message('ros:mlros2:serviceserver:DuplicateSvcServerHandlesError'))
            end
            svcServerInfo = nodeInfo.svcservers(whichServer);
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ros2svcserver';
        end
    end
end
