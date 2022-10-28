classdef ros2svcclient < ros.internal.mixin.InternalAccess & ...
        coder.ExternalDependency
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
%#codegen

    properties (Dependent, SetAccess = private)
        %RequestMessage
        RequestMessage

        %ResponseMessage
        ResponseMessage
    end

    properties (SetAccess = immutable)
        %ServiceType - The type of the service
        ServiceType

        %ServiceName - The name of the service
        ServiceName

        %RequestType - Message type of the service request
        RequestType

        %ResponseType - Message type of the service response
        ResponseType

        %History - The message queue mode
        History

        %Depth - The message queue size
        Depth

        %Reliability - The delivery guarantee of messages
        Reliability

        %Durability - The persistence of messages
        Durability
    end

    properties (Access = private)
        %Args - Function arguments for NewRequestFcn
        Args
        ReqMsgStruct
        RespMsgStruct
    end

    properties
        SvcClientHelperPtr
    end

    methods
        function obj = ros2svcclient(node, serviceName, serviceType, varargin)
        %ros2svcclient Constructor
        %   Attach a new service client to the given ROS 2 node. The "name"
        %   and "type" arguments are required and specifies the service
        %   to which this client should connect. Please see the class
        %   documentation (help ros2svcclient) for more details.

        % Parse the inputs to the constructor

            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            coder.internal.narginchk(3,11,nargin);

            % Validate input ros2node
            validateattributes(node,{'ros2node'},{'scalar'}, ...
                               'ros2svcclient','node');
            % Service name and type must be specified for codegen
            svcname = convertStringsToChars(serviceName);
            validateattributes(svcname,{'char'},{'nonempty'}, ...
                               'ros2svcclient','serviceName');
            svctype = convertStringsToChars(serviceType);
            validateattributes(svctype,{'char'},{'nonempty'}, ...
                               'ros2svcclient','servoceType');

            % Parse NV pairs
            nvPairs = struct('History',uint32(0),...
                             'Depth',uint32(0),...
                             'Reliability',uint32(0),...
                             'Durability',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{:});

            qosHistory = coder.internal.getParameterValue(pStruct.History,'keeplast',varargin{:});
            validateStringParameter(qosHistory,{'keeplast', 'keepall'},'ros2svcclient','History');

            qosDepth = coder.internal.getParameterValue(pStruct.Depth,1,varargin{:});
            validateattributes(qosDepth,{'numeric'},...
                               {'scalar','nonempty','integer','nonnegative'},...
                               'ros2svcclient','Depth');

            qosReliability = coder.internal.getParameterValue(pStruct.Reliability,'reliable',varargin{:});
            validateStringParameter(qosReliability,{'reliable', 'besteffort'},'ros2svcclient','Reliability');

            qosDurability = coder.internal.getParameterValue(pStruct.Durability,'volatile',varargin{:});
            validateStringParameter(qosDurability,{'transientlocal', 'volatile'},'ros2svcclient','Durability');

            % Store input arguments
            obj.ServiceName = svcname;
            obj.ServiceType = svctype;
            obj.RequestType = [svctype 'Request'];
            obj.ResponseType = [svctype 'Response'];
            obj.History = convertStringsToChars(qosHistory);
            obj.Depth = qosDepth;
            obj.Reliability = convertStringsToChars(qosReliability);
            obj.Durability = convertStringsToChars(qosDurability);

            qos_profile = coder.opaque('rmw_qos_profile_t', ...
                                       'rmw_qos_profile_default', 'HeaderFile', 'rmw/qos_profiles.h');
            qos_profile = ros.ros2.internal.setQOSProfile(qos_profile, obj.History, obj.Depth, ...
                                                          obj.Reliability, obj.Durability);

            % Get and register code generation information
            cgReqInfo = coder.const(@ros.codertarget.internal.getCodegenInfo, svcname, [svctype 'Request'], 'svc', 'ros2');
            reqMsgStructGenFcn = str2func(cgReqInfo.MsgStructGen);
            obj.ReqMsgStruct = reqMsgStructGenFcn(); % Setup return type for service request message

            cgRespInfo = coder.const(@ros.codertarget.internal.getCodegenInfo, svcname, [svctype 'Response'], 'svc', 'ros2');
            respMsgStructGenFcn = str2func(cgRespInfo.MsgStructGen);
            obj.RespMsgStruct = respMsgStructGenFcn(); % Setup return type for service response message

            % Get and register code generation information
            cgReqInfo = coder.const(@ros.codertarget.internal.getCodegenInfo, svcname, [svctype 'Request'], 'svc', 'ros2');
            reqMsgStructGenFcn = str2func(cgReqInfo.MsgStructGen);
            obj.ReqMsgStruct = reqMsgStructGenFcn(); % Setup return type for service request message

            cgRespInfo = coder.const(@ros.codertarget.internal.getCodegenInfo, svcname, [svctype 'Response'], 'svc', 'ros2');
            respMsgStructGenFcn = str2func(cgRespInfo.MsgStructGen);
            obj.RespMsgStruct = respMsgStructGenFcn(); % Setup return type for service response message

            % Create pointer to MATLABROS2SvcClient object
            coder.ceval('auto reqStructPtr = ', coder.wref(obj.ReqMsgStruct));
            coder.ceval('auto respStructPtr = ', coder.wref(obj.RespMsgStruct));

            TemplateTypeStr = ['MATLABROS2SvcClient<',cgReqInfo.CppSvcType, ...
                               ',' cgReqInfo.MsgClass ',' cgRespInfo.MsgClass ...
                               ',' cgReqInfo.MsgStructGen '_T,' cgRespInfo.MsgStructGen '_T>'];

            % Create service client, note that there is no need to
            % check CppPreserveClasses for service client creation
            % since there is no callback function.
            obj.SvcClientHelperPtr = coder.opaque(['std::unique_ptr<', TemplateTypeStr, '>'], 'HeaderFile', 'mlros2_svcclient.h');
            obj.SvcClientHelperPtr = coder.ceval(['std::unique_ptr<', TemplateTypeStr, ...
                                                  '>(new ', TemplateTypeStr, '(reqStructPtr, respStructPtr));//']);

            coder.ceval('MATLABROS2SvcClient_createSvcClient',obj.SvcClientHelperPtr, ...
                        node.NodeHandle, coder.rref(obj.ServiceName), ...
                        size(obj.ServiceName,2), qos_profile);
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

            coder.inline('never');

            % Warning if no status output
            if nargout<2
                coder.internal.compileWarning('ros:mlros2:codegen:MissingStatusOutput','call');
            end

            % Default outputs
            response = ros2message(obj.ResponseType);
            status = false;
            statusText = 'unknown';

            % If no varargin, use default service request message
            indx = 1;
            if nargin<2 || isstring(varargin{1}) || ischar(varargin{1})
                obj.ReqMsgStruct = ros2message(obj);
            else
                % The first argument in varargin should always be service
                % request message struct
                obj.ReqMsgStruct = varargin{1};
                indx = indx + 1;
            end

            nvPairs = struct('Timeout',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});
            callTimeout = coder.internal.getParameterValue(pStruct.Timeout,inf,varargin{indx:end});

            % Runtime verification for input request message and input
            % timeout, return right away with statusText set to 'input'.
            % Note that we only do this if there is more than one output.
            if ((~strcmp(obj.ReqMsgStruct.MessageType, obj.RequestType)) || (callTimeout < 0)) && nargout > 1
                statusText = 'input';
                return;
            end

            % Compile time error check
            validateattributes(callTimeout,{'numeric'},...
                               {'scalar','nonempty','real','positive'},'Call','Timeout');

            % Convert callTimout from s to ms
            if callTimeout < 0
                coder.internal.error('ros:mlros2:serviceclient:CallWaitTimeout',sprintf('%.2f', double(callTimeout)));
            end

            % Address synatx: call(client,"Timeout",inf)
            % Since MATLAB Interpretation mode does not allow "0" as input
            % timeout, "0" will be passed to C++ class representing
            % infinite case.
            if isinf(callTimeout)
                callTimeout = 0;
            end

            callTimeoutMS = floor(callTimeout * 1000);
            statusIndex = 2;
            coder.ceval('MATLABROS2SvcClient_callService',obj.SvcClientHelperPtr, ...
                        callTimeoutMS, coder.ref(statusIndex));

            % Retrieve staus and statusText from statusIndex
            if isequal(statusIndex, 0)
                % Receive response within specified timeout
                status = true;
                statusText = 'success';
            elseif isequal(statusIndex, 1)
                statusText = 'timeout';
            end

            % Return response based on status
            statusIndicator = status;
            if ~statusIndicator && nargout<2
                coder.internal.error('ros:mlros2:serviceclient:CallWaitTimeout',sprintf('%.2f', double(callTimeout)));
            else
                % Ignore warning here since this could be reached.
                % MATLAB cannot detect status update when it was passed as
                % a reference to an external cpp function.
                response = obj.ResponseMessage;
            end
        end

        function status = isServerAvailable(obj)
        % ISSERVERAVAILABLE Check whether service server is available
        %   STATUS = ISSERVERAVAILABLE(CLIENT) checks whether a service
        %   server is connected to the client and ready for communication.

        % Initialize status as false
            status = false;
            coder.ceval('MATLABROS2SvcClient_isServerAvailable',obj.SvcClientHelperPtr, coder.wref(status));
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

            coder.inline('never');

            % Warning if no status output
            if nargout<1
                coder.internal.compileWarning('ros:mlros2:codegen:MissingStatusOutput','waitForServer');
            end

            % Initialize status as false
            status = false;
            % Parse name-value pair
            nvPairs = struct('Timeout',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{:});
            waitTimeout = coder.internal.getParameterValue(pStruct.Timeout,inf,varargin{:});

            % Runtime verification for input timeout, return right away if
            % user request output and the input timeout is invalid.
            if waitTimeout < 0 && nargout > 0
                statusText = 'input';
                varargout = {status, statusText};
                return;
            end
            validateattributes(waitTimeout,{'numeric'},...
                               {'scalar','real','positive'},'waitForServer','Timeout');

            % Address synatx: waitForServer(client,"Timeout",inf)
            % Since MATLAB Interpretation mode does not allow "0" as input
            % timeout, "0" will be passed to C++ class representing
            % infinite case.
            if isinf(waitTimeout)
                waitTimeout = 0;
            end

            waitTimeoutMS = floor(waitTimeout * 1000);
            coder.ceval('MATLABROS2SvcClient_waitForService',obj.SvcClientHelperPtr, ...
                        waitTimeoutMS, coder.ref(status));

            statusIndicator = status;
            if ~statusIndicator && nargout<1
                % Throw runtime error if runtime error check is on
                % Writing this separately to avoid optimizing away
                % statusText assignment when RunTimeCheck is off
                coder.internal.error('ros:mlros2:serviceclient:ConnectWaitTimeout',obj.ServiceName,sprintf('%.2f', double(waitTimeout)));
            end

            if status
                % This can be reached since the value will be updated with
                % coder.ceval
                statusText = 'success'; %#ok<UNRCH>
            else
                statusText = 'timeout';
            end

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

        function msg = get.RequestMessage(obj)

            coder.ceval('MATLABROS2SvcClient_lock',obj.SvcClientHelperPtr);
            msg = obj.ReqMsgStruct;
            coder.ceval('MATLABROS2SvcClient_unlock',obj.SvcClientHelperPtr);
        end

        function msg = get.ResponseMessage(obj)

            coder.ceval('MATLABROS2SvcClient_lock',obj.SvcClientHelperPtr);
            msg = obj.RespMsgStruct;
            coder.ceval('MATLABROS2SvcClient_unlock',obj.SvcClientHelperPtr);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'RequestType','ResponseType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS 2 SvcClient';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlros2_svcclient.h',srcFolder);
                addIncludeFiles(buildInfo,'mlros2_qos.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'ServiceType','ServiceName',...
                     'RequestType','ResponseType'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
