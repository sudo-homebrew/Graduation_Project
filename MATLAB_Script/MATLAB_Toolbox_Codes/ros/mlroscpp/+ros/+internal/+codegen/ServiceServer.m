classdef ServiceServer < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
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
%      NewRequestFcn  - (Read-Only) Callback property for service request callbacks
%
%   ServiceServer methods:
%      rosmessage     - Create a new service response message
%
%   Example:
%
%      % Create a new service server and assign a callback with user
%      % data in the constructor
%      % Use message structs with this server
%      userData = randi(20);
%      server = ros.ServiceServer(node,"/gazebo/get_model_state2",...
%          "gazebo_msgs/GetModelState",{@fcn2, userData},...
%          "DataFormat","struct");
%
%   See also ROSSVCSERVER, ros.ServiceClient, ROSSERVICE.

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
    end

    properties (Access = private)
        %Args - Function arguments for NewRequestFcn
        Args
        ReqMsgStruct
        RespMsgStruct
        SvcServerHelper
        IsInitialized = false
    end

    methods
        function obj = ServiceServer(node, serviceName, serviceType, varargin)
        %ServiceServer Constructor
        %   Attach a new service server to the ROS node object. The
        %   "name" and "type" arguments are required and specify the
        %   advertised service name and type. Please see the class
        %   documentation (help ros.ServiceServer) for more details.

            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            % Ensure serviceType is not empty
            coder.internal.assert(nargin>2 && contains(serviceType,'/'),'ros:mlroscpp:codegen:MissingMessageType',serviceName,'ServiceServer');

            % A node cannot create another node in codegen
            if ~isempty(node)
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Service name and type must be specified for codegen.
            svcname = convertStringsToChars(serviceName);
            validateattributes(svcname,{'char'},{'nonempty'}, ...
                               'ServiceServer','serviceName');
            svctype = convertStringsToChars(serviceType);
            validateattributes(svctype,{'char'},{'nonempty'}, ...
                               'ServiceServer','serviceType');

            % Extract callback function if specified
            indx = 1;
            if nargin > 4
                if isa(varargin{1},'function_handle')
                    obj.NewRequestFcn = varargin{1};
                    indx = indx + 1;
                elseif iscellstr(varargin{1})
                    cb = varargin{1};
                    obj.NewRequestFcn = cb{1};
                    obj.Args = cb(2:end);
                    indx = indx + 1;
                end
            end

            % Parse NV pairs
            nvPairs = struct('DataFormat',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});
            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{indx:end});
            validateStringParameter(dataFormat,{'object','struct'},'ServiceServer','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'), ...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','ServiceServer');

            % Store input arguments
            obj.ServiceName = svcname;
            obj.ServiceType = svctype;
            obj.RequestType = [svctype 'Request'];
            obj.ResponseType = [svctype 'Response'];
            obj.DataFormat = dataFormat;

            % Get and register code generation information
            cgReqInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,svcname,[svctype 'Request'],'svc');
            reqMsgStructGenFcn = str2func(cgReqInfo.MsgStructGen);
            obj.ReqMsgStruct = reqMsgStructGenFcn(); % Setup return type for service request message

            cgRespInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,svcname,[svctype 'Response'],'svc');
            respMsgStructGenFcn = str2func(cgRespInfo.MsgStructGen);
            obj.RespMsgStruct = respMsgStructGenFcn(); % Setup return type for service response message

            % Create an instance of MATLABSvcServer object
            coder.ceval([cgReqInfo.MsgClass '* reqMsgPtr = nullptr;//']);
            coder.ceval([cgRespInfo.MsgClass '* respMsgPtr = nullptr;//']);
            coder.ceval('auto reqStructPtr = ', coder.wref(obj.ReqMsgStruct));
            coder.ceval('auto respStructPtr = ', coder.wref(obj.RespMsgStruct));

            templateTypeStr = ['MATLABSvcServer<', ...
                               cgReqInfo.MsgClass ',' cgRespInfo.MsgClass ',' ...
                               cgReqInfo.MsgStructGen '_T,' cgRespInfo.MsgStructGen '_T>'];

            obj.SvcServerHelper = coder.opaque(['std::unique_ptr<', templateTypeStr, '>'],'HeaderFile','mlroscpp_svcserver.h');
            if ros.internal.codegen.isCppPreserveClasses
                % Create ServiceServer by passing in class method as
                % callback
                obj.SvcServerHelper = coder.ceval(['std::unique_ptr<' templateTypeStr, ...
                                                   '>(new ', templateTypeStr, '([this](){this->callback();},', ...
                                                   'reqStructPtr,respStructPtr));//']);
            else
                % Create ServiceServer by passing in static function
                % as callback
                obj.SvcServerHelper = coder.ceval(['std::unique_ptr<' templateTypeStr, ...
                                                   '>(new ', templateTypeStr, '([obj](){ServiceServer_callback(obj);},', ...
                                                   'reqStructPtr,respStructPtr));//']);
            end
            coder.ceval('MATLABSvcServer_createSvcServer', obj.SvcServerHelper, coder.rref(obj.ServiceName), ...
                        size(obj.ServiceName,2));

            % Ensure callback is not optimized away by making an explicit
            % call here
            obj.callback();
            obj.IsInitialized = true;
        end

        function callback(obj)
            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.RequestType);
            if ~isempty(obj.NewRequestFcn) && (obj.IsInitialized)
                % Call user defined callback function
                latestReqMsg = obj.RequestMessage;
                latestRespMsg = obj.ResponseMessage;
                if isempty(obj.Args)
                    obj.RespMsgStruct = obj.NewRequestFcn(obj,latestReqMsg,latestRespMsg);
                else
                    obj.RespMsgStruct = obj.NewRequestFcn(obj,latestReqMsg,latestRespMsg,obj.Args{:});
                end
            end
        end

        function msg = rosmessage(obj)
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

            coder.inline('never');
            msg = rosmessage(obj.ResponseType,'DataFormat','struct');
        end

        function msg = get.RequestMessage(obj)
            coder.ceval('MATLABSvcServer_lock',obj.SvcServerHelper);
            msg = obj.ReqMsgStruct;
            coder.ceval('MATLABSvcServer_unlock',obj.SvcServerHelper);
        end

        function msg = get.ResponseMessage(obj)
            coder.ceval('MATLABSvcServer_lock',obj.SvcServerHelper);
            msg = obj.RespMsgStruct;
            coder.ceval('MATLABSvcServer_unlock',obj.SvcServerHelper);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'ResponseType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS SvcServer';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_svcserver.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'NewRequestFcn','ServiceType','ServiceName',...
                     'DataFormat','RequestType','ResponseType'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
