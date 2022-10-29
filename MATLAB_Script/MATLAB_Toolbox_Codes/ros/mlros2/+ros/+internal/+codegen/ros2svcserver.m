classdef ros2svcserver < ros.internal.mixin.InternalAccess & ...
        coder.ExternalDependency
% ros2svcserver - Code generation equivalent for ros2svcserver
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
%#codegen

    properties (Dependent, SetAccess = private)
        %RequestMessage
        RequestMessage

        %ResponseMessage
        ResponseMessage
    end

    properties (SetAccess = immutable)
        %NewRequestFcn - Callback property for service request callbacks
        NewRequestFcn

        %ServiceType - The type of the service
        ServiceType

        %ServiceName - The name of the service
        ServiceName

        %DataFormat - Message format of the service server
        DataFormat

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
        IsInitialized = false
    end

    properties
        SvcServerHelperPtr
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

            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            coder.internal.narginchk(4,12,nargin);

            % Validate input ros2node
            validateattributes(node,{'ros2node'},{'scalar'}, ...
                               'ros2svcserver','node');
            % Service name and type must be specified for codegen
            svcname = convertStringsToChars(serviceName);
            validateattributes(svcname,{'char'},{'nonempty'}, ...
                               'ros2svcserver','serviceName');
            svctype = convertStringsToChars(serviceType);
            validateattributes(svctype,{'char'},{'nonempty'}, ...
                               'ros2svcserver','serviceType');

            % Extract callback function
            validateattributes(cb,{'function_handle','cell'},{'nonempty'}, ...
                               'ros2svcserver','cb');
            if isa(cb, 'function_handle')
                obj.NewRequestFcn = cb;
            elseif iscellstr(cb) %#ok<ISCLSTR>
                obj.NewRequestFcn = cb{1};
                obj.Args = cb(2:end);
            end

            % Parse NV pairs
            nvPairs = struct('History',uint32(0),...
                             'Depth',uint32(0),...
                             'Reliability',uint32(0),...
                             'Durability',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{:});

            qosHistory = coder.internal.getParameterValue(pStruct.History,'keeplast',varargin{:});
            validateStringParameter(qosHistory,{'keeplast', 'keepall'},'ros2svcserver','History');

            qosDepth = coder.internal.getParameterValue(pStruct.Depth,1,varargin{:});
            validateattributes(qosDepth,{'numeric'},...
                               {'scalar','nonempty','integer','nonnegative'},...
                               'ros2svcserver','Depth');

            qosReliability = coder.internal.getParameterValue(pStruct.Reliability,'reliable',varargin{:});
            validateStringParameter(qosReliability,{'reliable', 'besteffort'},'ros2svcserver','Reliability');

            qosDurability = coder.internal.getParameterValue(pStruct.Durability,'volatile',varargin{:});
            validateStringParameter(qosDurability,{'transientlocal', 'volatile'},'ros2svcserver','Durability');

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

            % Create pointer to MATLABROS2SvcServer object
            coder.ceval('auto reqStructPtr = ', coder.wref(obj.ReqMsgStruct));
            coder.ceval('auto respStructPtr = ', coder.wref(obj.RespMsgStruct));

            TemplateTypeStr = ['MATLABROS2SvcServer<',cgReqInfo.CppSvcType, ...
                               ',' cgReqInfo.MsgClass ',' cgRespInfo.MsgClass ...
                               ',' cgReqInfo.MsgStructGen '_T,' cgRespInfo.MsgStructGen '_T>'];

            obj.SvcServerHelperPtr = coder.opaque(['std::unique_ptr<', TemplateTypeStr, '>'], 'HeaderFile', 'mlros2_svcserver.h');
            if ros.internal.codegen.isCppPreserveClasses
                % Create service server by passing in class method as callback
                obj.SvcServerHelperPtr = coder.ceval(['std::unique_ptr<', TemplateTypeStr, ...
                                                      '>(new ', TemplateTypeStr, '(reqStructPtr, respStructPtr, [this]{this->callback();}));//']);
            else
                % Create service server by passing in static function as callback
                obj.SvcServerHelperPtr = coder.ceval(['std::unique_ptr<', TemplateTypeStr, ...
                                                      '>(new ', TemplateTypeStr, '(reqStructPtr, respStructPtr, [obj]{ros2svcserver_callback(obj);}));//']);
            end
            coder.ceval('MATLABROS2SvcServer_createSvcServer',obj.SvcServerHelperPtr, ...
                        node.NodeHandle, coder.rref(obj.ServiceName), ...
                        size(obj.ServiceName,2), qos_profile);

            % Ensure callback is not optimized away by making an explicit
            % call here
            obj.callback();
            obj.IsInitialized = true;
        end

        function callback(obj)
            coder.inline('never');
            if ~isempty(obj.NewRequestFcn) && (obj.IsInitialized)
                % Call user defined callback function
                if isempty(obj.Args)
                    obj.RespMsgStruct = obj.NewRequestFcn(obj.ReqMsgStruct,obj.RespMsgStruct);
                else
                    obj.RespMsgStruct = obj.NewRequestFcn(obj.ReqMsgStruct,obj.RespMsgStruct,obj.Args{:});
                end
            end
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

        function msg = get.RequestMessage(obj)

            coder.ceval('MATLABROS2SvcServer_lock',obj.SvcServerHelperPtr);
            msg = obj.ReqMsgStruct;
            coder.ceval('MATLABROS2SvcServer_unlock',obj.SvcServerHelperPtr);
        end

        function msg = get.ResponseMessage(obj)

            coder.ceval('MATLABROS2SvcServer_lock',obj.SvcServerHelperPtr);
            msg = obj.RespMsgStruct;
            coder.ceval('MATLABROS2SvcServer_unlock',obj.SvcServerHelperPtr);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'ResponseType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS 2 SvcServer';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlros2_svcserver.h',srcFolder);
                addIncludeFiles(buildInfo,'mlros2_qos.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'NewRequestFcn','ServiceType','ServiceName',...
                     'RequestType','ResponseType'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
