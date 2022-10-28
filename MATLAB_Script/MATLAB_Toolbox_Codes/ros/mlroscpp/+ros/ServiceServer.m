classdef ServiceServer < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
%ServiceServer Create a ROS service server object
%   Use ServiceServer to create a ROS service server that can receive
%   requests from, and send responses to, a ROS service client.
%   The service server must exist before creating the service client.
%   When you create the client, it establishes a connection to the
%   server. The connection persists while both client and server exist
%   and can reach each other.
%
%   When you create the service server, it registers itself with the
%   ROS master.  To get a list of services that are available on the
%   current ROS network, or to get more information about any particular
%   service, use the rosservice function.
%
%   The service is defined by a type and a pair of messages: one for the
%   request and one for the response. The service server will receive a
%   request, construct an appropriate response, and return it to the
%   client. The behavior of the service server is inherently asynchronous,
%   as it only becomes active when a service client connects and issues
%   a call.
%
%   SERVER = ros.ServiceServer(NODE,SVCNAME,SVCTYPE) creates and returns a
%   service server object. The service will be available in the ROS network
%   through its name SVCNAME and has the type SVCTYPE
%   However, the service object cannot respond to service requests until
%   a function handle callback is specified. The service server will be
%   attached to the ros.Node object NODE. SVCNAME and SVCTYPE are
%   string scalars.
%
%   SERVER = ros.ServiceServer(NODE,SVCNAME,SVCTYPE,CB) creates and returns
%   a service server object with SVCNAME and SVCTYPE. It also specifies
%   the function handle callback, CB, that constructs a response when
%   the server receives a request. CB can be a single function handle
%   or a cell array. The first element of the cell array must be a
%   function handle, or a string containing the name of a function.
%   The remaining elements of the cell array can be arbitrary user data
%   that is passed to the callback function.
%
%   SERVER = ros.ServiceServer(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments.
%
%      "DataFormat" - Determines format of ROS message provided to the
%                     NewRequestFcn callback.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   The service server callback function requires at least three input
%   arguments and one output. The first argument, SRC, is the associated
%   service server object. The second argument, REQUEST, is the request
%   message object sent by the service client. The third argument is
%   the default response message object, DEFAULTRESPONSE.  Use the
%   DEFAULTRESPONSE as a starting point for constructing the function
%   output RESPONSE, which will be sent back to the service client
%   after the function returns.
%
%      function RESPONSE = serviceCallback(SRC,REQUEST,DEFAULTRESPONSE)
%          RESPONSE = DEFAULTRESPONSE;
%          % Build the response message here
%      end
%
%   While setting the callback, to construct a callback that accepts
%   additional parameters, use a cell array that includes both the
%   function handle callback and the parameters.
%
%
%   ServiceServer properties:
%      ServiceName    - (Read-Only) The name of the service
%      ServiceType    - (Read-Only) The type of the service
%      DataFormat     - (Read-Only) Message format provided by the server
%      NewRequestFcn  - Callback property for service request callbacks
%
%   ServiceServer methods:
%      rosmessage     - Create a new service response message
%
%
%   Example:
%
%      % Create a service server
%      % This assumes that there is an existing node object
%      % Use struct messages for better performance
%      server = ros.ServiceServer(node,"/gazebo/get_model_state",...
%         "gazebo_msgs/GetModelState","DataFormat","struct");
%
%      % Assign a callback for incoming service calls
%      server.NewRequestFcn = @fcn1;
%
%      % Create a new service server with a different name and assign a
%      % callback with user data in the constructor
%      % Use message objects with this server
%      userData = randi(20);
%      server2 = ros.ServiceServer(node,"/gazebo/get_model_state2",...
%          "gazebo_msgs/GetModelState",{@fcn2, userData},...
%          "DataFormat","object");
%
%   See also ROSSVCSERVER, ros.ServiceClient, ROSSERVICE.

%   Copyright 2014-2021 The MathWorks, Inc.

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

    properties (Transient, Access = {?ros.internal.mixin.ROSInternalAccess,...
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
        %   combination, as that limits the ROS processing queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default.
        MaxConcurrentCallbacks
    end

    methods
        function obj = ServiceServer(node, serviceName, serviceType, varargin)
        %ServiceServer Constructor
        %   Attach a new service server to the given ROS node. The
        %   "name" and "type" arguments are required and specify the
        %   advertised service name and type. Please see the class documentation
        %   (help ros.ServiceServer) for more details.

        % If no node specified, use the global node.
            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Parse the inputs to the constructor
            [serviceName, serviceType, varargin{:}] = ...
                convertStringsToChars(serviceName, serviceType, varargin{:});
            % Special handling for empty callback
            if nargin > 3 && isempty(varargin{1})
                varargin{1} = function_handle.empty;
            end
            parser = getParser(obj);
            parse(parser, node, serviceName, serviceType, varargin{:})
            node = parser.Results.node;
            resolvedName = resolveName(node, parser.Results.serviceName);
            serviceType = parser.Results.serviceType;
            requestFcn = parser.Results.requestFcn;

            % Make sure that service does not exist yet
            if ros.internal.NetworkIntrospection.isServiceAvailable(...
                resolvedName, node.MasterURI)
                error(message('ros:mlros:serviceserver:AlreadyExists', resolvedName));
            end

            % Make sure that service type is valid
            svcList = rostype.getServiceList;
            invalidServiceTypes = ros.internal.utilities.getServiceTypesWithCppKeyword;
            if ~ismember(serviceType,svcList)
                if ismember(serviceType,invalidServiceTypes)
                    error(message(...
                        'ros:utilities:message:ServiceNotSupportedInThisRelease',serviceType));
                else
                    error(message('ros:mlros:serviceserver:InvalidType', serviceType));
                end
            end

            % Set some object properties
            obj.ServiceType = serviceType;
            obj.RequestType = [serviceType 'Request'];
            obj.ResponseType = [serviceType 'Response'];
            setDataFormat(obj, parser.Results.DataFormat)

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode; %ros.internal.Node
            obj.ServerNodeHandle = node.ServerNodeHandle; %node handle for server
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Get service info
            obj.ServiceInfo = ros.internal.ros.getServiceInfo(obj.RequestType, ...
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
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:serviceserver:CreateError', resolvedName, serviceType));
                throw(newEx.addCustomCause(ex));
            end

            % Store the service server handle in node object
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);

            % Create the service server object
            createSvcServer(obj, resolvedName);

            % Get other info after the service is created in ros network
            obj.ServiceInfo.rosInfo = ...
                ros.internal.NetworkIntrospection.getServiceInfo(resolvedName, node.MasterURI);
        end

        function delete(obj)
        %DELETE Shut down service server
        %   DELETE(SERVER) shuts down the ROS service server object SERVER
        %   and removes its registration from the ROS master

            obj.NewRequestFcn = []; % Immediately halt callbacks

            % Cannot tell server to remove the service client without valid
            % internal node and server handle value
            if ~isempty(obj.InternalNode) && ...
                    isvalid(obj.InternalNode) && ...
                    ~isempty(obj.SvcServerHandle)

                try
                    removeSvcServer(obj.InternalNode, ...
                                    obj.SvcServerHandle);
                catch
                    warning(message('ros:mlros:serviceserver:ShutdownError'));
                end
            end
            obj.InternalNode = [];
        end

        function msg = rosmessage(obj, varargin)
        % ROSMESSAGE Create a new service response message
        %   MSG = ROSMESSAGE(SERVER) creates and returns an empty message MSG.
        %   The message type of MSG is determined by the service type. The
        %   format of MSG is determined by the DataFormat of the service
        %   server. The message is the default response that this server
        %   can use to reply to client requests.
        %
        %   Example:
        %      % Create a service server and response message
        %      server1 = ros.ServiceServer(node,"/gazebo/get_model_state",...
        %         "gazebo_msgs/GetModelState");
        %      msgObj = ROSMESSAGE(server1);
        %
        %      % Improve performance by using struct messages
        %      server2 = ros.ServiceServer(node,"/gazebo/get_model_state",...
        %         "gazebo_msgs/GetModelState","DataFormat","struct");
        %      msgStruct = ROSMESSAGE(server2);

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.ResponseType, 'DataFormat', obj.DataFormat);
        end

        function set.NewRequestFcn(obj, requestFcn)
        %set.NewRequestFcn Set the callback for new service requests
        %
        %   sub.NewRequestFcn = CB sets the callback function that should
        %   be invoked when a new service request is received. Here, CB is
        %   either a scalar function handle or a cell array. You can
        %   pass additional parameters to the callback function by including
        %   both the function handle and the parameters as elements
        %   of a cell array and assign it to CB. If no callbacks should
        %   be executed, assign the empty matrix [] to CB.
        %
        %   Each callback function must have the
        %   following signature (see the help for ros.ServiceServer for
        %   a more detailed explanation):
        %
        %      function RESPONSE = serviceCallback(SRC, REQUEST, DEFAULTRESPONSE)
        %          RESPONSE = DEFAULTRESPONSE;
        %          % Build the response message here
        %      end


        % Make sure this is a valid function handle
            if ~isempty(requestFcn)
                [funcHandle, userData] = ...
                    ros.internal.Parsing.validateFunctionHandle(requestFcn);
                obj.ActualNewRequestFcn = funcHandle; %#ok<MCSUP>
                obj.NewRequestCallbackArgs = userData; %#ok<MCSUP>
            else
                obj.ActualNewRequestFcn = function_handle.empty; %#ok<MCSUP>
                obj.NewRequestCallbackArgs = {}; %#ok<MCSUP>
            end
            obj.NewRequestFcn = requestFcn;
        end
    end

    methods (Access = ?ros.internal.mixin.ROSInternalAccess)
        function processRequest(obj, requestMsg, varargin)
        % processRequest takes action based on new request from client

        % Call the callback function if assigned
            if ~isempty(obj.NewRequestFcn)

                % Default response
                response = rosmessage(obj);
                % Use response type to determine data format for
                % request message when passed to user callback
                if obj.UseObjectMsg
                    requestMsg = feval(obj.ServiceInfo.msgClassGen, ...
                                       requestMsg);
                end

                try
                    % Evaluate the response
                    response = feval(obj.ActualNewRequestFcn, ...
                                     obj, ...
                                     requestMsg, ...
                                     response, ...
                                     obj.NewRequestCallbackArgs{:});
                catch ex
                    % Send error message back to back-end if something goes
                    % wrong due to exception or user mistake in callback
                    warning(message('ros:mlros:serviceserver:UserCallbackError', ex.message))
                    errorResponse.message = ex.message;
                    sendBackErrorResponse(obj, errorResponse);
                end

                % Convert response to struct if needed
                if obj.UseObjectMsg
                    response = toROSStruct(response);
                end

                % Send the response back over the network
                try
                    sendBackResponse(obj, response);
                catch ex
                    % Send error message back to back-end if something goes
                    % wrong due to exception or user mistake in callback
                    try
                        validateInputMessage(obj, response, obj.ResponseType, 'ServiceServer', 'callback')
                    catch ex
                        % Exception will be used in response message
                    end
                    warning(message('ros:mlros:serviceserver:SendResponseError', ex.message))
                    errorResponse.message = ex.message;
                    sendBackErrorResponse(obj, errorResponse);
                end
            else
                % Send error message back to server if no call-back is
                % defined by user and release the mutex-lock in backend

                errorResponse.message = 'Callback is empty';
                sendBackErrorResponse(obj, errorResponse);
            end
        end
    end

    methods (Access = private)
        function createSvcServer(obj,varargin)
        %creates service-server on ROS network
            callbackFcn = obj.ProcessRequestCallbackHandler.CallbackName;

            % As both ROS1 and ROS2 uses same MCOS API and ROS2-Services
            % has QOS, while using the same API for ROS1, a dummy value is
            % being sent as QOS to backend which is not going o be used
            % anywhere.
            dummyQOS = struct();
            dllPathsRequest = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Request'],'ros');
            dllPathsResponse = ros.internal.utilities.getPathOfDependentDlls([obj.ServiceType 'Response'],'ros');
            dllPaths = [dllPathsRequest dllPathsResponse];

            try
                returnCall = addSvcServer(obj.InternalNode, ...
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
                obj.SvcServerHandle = returnCall.handle;
                obj.ServiceName = returnCall.svcName;
                % Initialize callback to process requests.
                initSvcServerCallback(obj.InternalNode, ...
                                      returnCall.handle, ...
                                      obj.ProcessRequestCallbackHandler, ...
                                      obj.MaxConcurrentCallbacks);
                % No need to check reply - should error on failure
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:serviceserver:CreateError', obj.ServiceName, obj.ServiceType));
                throw(newEx.addCustomCause(ex));
            end
        end

        function sendBackResponse(obj,response)
        %sends back the response to server.

            responseFromMLtoSvcServer(obj.InternalNode, ...
                                      obj.SvcServerHandle, ...
                                      response);
        end

        function sendBackErrorResponse(obj,response)
        % sends the error-response back when exception occurred or user
        % has done some mistake in the callback.
        % This unlocks the mutex lock in backend-server because server is not
        % going to get the response back from the callback in case of exception.

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
                        validateattributes(x, {'ros.Node'}, ...
                                           {'scalar', 'nonempty'}, ...
                                           'ServiceServer', 'node'));
            addRequired(parser, 'serviceName',  @(x) ...
                        validateattributes(x, {'char'}, {'nonempty'}, ...
                                           'ServiceServer', 'svcName'));
            addRequired(parser, 'serviceType',  @(x) ...
                        validateattributes(x, {'char'}, {'nonempty'}, ...
                                           'ServiceServer', 'svcType'));
            % Need at least class validation on optional callback argument
            % to distinguish it from name-value pairs for the parser
            addOptional(parser, 'requestFcn', function_handle.empty, ...
                        @(x) validateattributes(x, ...
                                                {'function_handle', 'cell'}, ...
                                                {}, ...
                                                'ServiceServer', 'cb'));
            addDataFormatToParser(obj, parser, 'ServiceServer')
        end
    end

    methods(Access = private, Static)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ServiceServer';
        end
    end
end
