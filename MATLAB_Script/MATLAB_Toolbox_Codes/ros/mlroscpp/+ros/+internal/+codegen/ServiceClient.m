classdef ServiceClient < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
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
%   CLIENT = ros.ServiceClient(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of ROS message to be used by
%                     the service client, and returned from rosmessage.
%                     Using structs can be faster than using message
%                     objects. For code generation, "struct" is
%                     required to be specified as the "DataFormat".
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
%      [response, status] = call(client,request);
%
%      % Send the service request and wait five seconds for a response
%      % (blocking, with timeout).
%      [response, status] = call(client,request,"Timeout",5);
%
%   See also ROSSVCCLIENT, ros.ServiceServer, ROSSERVICE.

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

        %DataFormat - Message format of the service client
        DataFormat

        %RequestType - Message type of the service request
        RequestType

        %ResponseType - Message type of the service response
        ResponseType
    end

    properties (Access = private)
        ReqMsgStruct
        RespMsgStruct
    end

    properties
        SvcClientHelperPtr
    end

    methods
        function obj = ServiceClient(node, serviceName, varargin)
        %ServiceClient constructor
        %   Attach a new service client to the ROS node. The "name"
        %   argument is requiered and specifies the service to which
        %   this client should connect. Please see the class
        %   documentation (help ros.ServiceClient) for more details.

            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            % Ensure varargin is not empty
            coder.internal.assert(nargin>2 && contains(varargin{1},'/'),'ros:mlroscpp:codegen:MissingMessageType',serviceName,'ServiceClient');

            % A node cannot create another node in codegen
            if ~isempty(node)
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Service name and type must be specified for codegen.
            svcname = convertStringsToChars(serviceName);
            validateattributes(svcname,{'char'},{'nonempty'}, ...
                               'ServiceClient','serviceName');
            svctype = convertStringsToChars(varargin{1});
            validateattributes(svctype,{'char'},{'nonempty'}, ...
                               'ServiceClient','serviceType');

            % Parse NV pairs
            nvPairs = struct('DataFormat',uint32(0),...
                             'Timeout',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{2:end});
            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{2:end});
            validateStringParameter(dataFormat,{'object','struct'},'ServiceClient','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'),...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','ServiceClient');
            connectTimeout = coder.internal.getParameterValue(pStruct.Timeout,0,varargin{2:end});
            validateattributes(connectTimeout,{'numeric'},...
                               {'scalar','nonempty','real','nonnegative'},'ServiceClient','Timeout');

            % Store input arguments
            obj.ServiceName = svcname;
            obj.ServiceType = svctype;
            obj.DataFormat = dataFormat;
            obj.RequestType = [svctype 'Request'];
            obj.ResponseType = [svctype 'Response'];

            % Get and register code generation information
            cgReqInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,svcname,[svctype 'Request'],'svc');
            reqMsgStructGenFcn = str2func(cgReqInfo.MsgStructGen);
            obj.ReqMsgStruct = reqMsgStructGenFcn(); % Setup return type for service request message

            cgRespInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,svcname,[svctype 'Response'],'svc');
            respMsgStructGenFcn = str2func(cgRespInfo.MsgStructGen);
            obj.RespMsgStruct = respMsgStructGenFcn(); % Setup return type for service response message

            % Create an instance of MATLABSvcClient object
            coder.ceval([cgReqInfo.CppSvcType '*svcPtr = nullptr;//']);
            coder.ceval([cgReqInfo.MsgClass '* reqMsgPtr = nullptr;//']);
            coder.ceval([cgRespInfo.MsgClass '* respMsgPtr = nullptr;//']);
            coder.ceval('auto reqStructPtr= ', coder.wref(obj.ReqMsgStruct));
            coder.ceval('auto respStructPtr= ', coder.wref(obj.RespMsgStruct));

            templateTypeStr = ['MATLABSvcClient<',cgReqInfo.CppSvcType, ...
                               ',' cgReqInfo.MsgClass ',' cgRespInfo.MsgClass ',' ...
                               cgReqInfo.MsgStructGen '_T,' cgRespInfo.MsgStructGen '_T>'];

            obj.SvcClientHelperPtr = coder.opaque(['std::unique_ptr<', templateTypeStr, '>'],'HeaderFile','mlroscpp_svcclient.h');
            obj.SvcClientHelperPtr = coder.ceval(['std::unique_ptr<' templateTypeStr, ...
                                                  '>(new ', templateTypeStr, '(reqStructPtr,respStructPtr));//']);

            coder.ceval('MATLABSvcClient_createSvcClient',obj.SvcClientHelperPtr,coder.rref(obj.ServiceName),...
                        size(obj.ServiceName,2));

            % Avoid optimizing away SvcClientHelperPtr
            obj.isServerAvailable;
        end

        function status = isServerAvailable(obj)
        % ISSERVERAVAILABLE Check whether service server is available
        %   STATUS = ISSERVERAVAILABLE(CLIENT) checks whether a service
        %   server with the same service name as this service client
        %   is available.

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.SvcClientHelperPtr);

            status = false;
            status = coder.ceval('MATLABSvcClient_mlExists',obj.SvcClientHelperPtr);
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

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.SvcClientHelperPtr);

            % Warning if no status output
            if nargout<1
                coder.internal.compileWarning('ros:mlroscpp:codegen:MissingStatusOutput','waitForServer');
            end

            status = false;

            % Parse name-value pair
            nvPairs = struct('Timeout',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{:});
            waitTimeout = coder.internal.getParameterValue(pStruct.Timeout,inf,varargin{:});

            % Runtime verification for input timeout, return right away if
            % user request output and the input timeout is invalid.
            if waitTimeout <= 0 && nargout > 0
                statusText = 'input';
                varargout = {status, statusText};
                return;
            end

            % Address synatx: waitForServer(client,"Timeout",inf)
            % Since MATLAB Interpretation mode does not allow "0" as input
            % timeout, "0" will be passed to C++ class representing
            % infinite case.
            if isinf(waitTimeout)
                waitTimeout = 0;
            end

            validateattributes(waitTimeout,{'numeric'},...
                               {'scalar','real','nonnegative'},'waitForServer','Timeout');
            coder.ceval('MATLABSvcClient_mlWaitForExistence',obj.SvcClientHelperPtr, waitTimeout, coder.ref(status));

            statusIndicator = status;
            if ~statusIndicator && nargout<1
                % Throw runtime error if runtime error check is on
                % Writing this separately to avoid optimizing away
                % statusText assignment when RunTimeCheck is off
                coder.internal.error('ros:mlros:serviceclient:ConnectWaitTimeout',obj.ServiceName,sprintf('%.2f', double(waitTimeout)));
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

        function [response,status,statusText] = call(obj, varargin)
        %CALL Call the service server and receive a response
        %   RESPONSE = CALL(CLIENT) sends a default service request message
        %   and waits for a service RESPONSE. The default service
        %   request message is an empty message of type CLIENT.ServiceType.
        %   If the RESPONSE does not return as expected, an error message
        %   will be displayed on terminal during runtime.
        %
        %   RESPONSE = CALL(CLIENT,REQUEST) sends a specific service request
        %   message, REQUEST, and waits for a service RESPONSE. The type
        %   and format of the message must match the ServiceType and
        %   DataFormat of the client. If the RESPONSE does not return as
        %   expected, an error message will be displayed on terminal during
        %   runtime.
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
        %       'success' - The service response was successfully received
        %       'input'   - The input to the function is invalid.
        %       'timeout' - The service response was not received within
        %                   the specified timeout.
        %       'unknown' - The service response was not received due to
        %                   unknown service server errors.
        %
        %   Use CALL to send a service request to a service server,
        %   and then waits to receive a response from that service server.
        %
        %   This function blocks MATLAB from running the current
        %   program until the service client receives a service response.

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.SvcClientHelperPtr);

            % Warning if no status output
            if nargout<2
                coder.internal.compileWarning('ros:mlroscpp:codegen:MissingStatusOutput','call');
            end

            % Default outputs
            response = rosmessage(obj.ResponseType,'DataFormat','struct');
            status = false;
            statusText = 'unknown';

            % If no varargin, use default service request message
            indx = 1;
            if nargin<2 || isstring(varargin{1}) || ischar(varargin{1})
                obj.ReqMsgStruct = rosmessage(obj);
            else
                % The first argument in varargin should always be service
                % request message struct
                coder.internal.assert(isstruct(varargin{1}),'ros:mlroscpp:codegen:NodeMustBeEmpty');
                coder.internal.assert(strcmp(varargin{1}.MessageType, obj.RequestType),'ros:mlroscpp:message:InputTypeMismatch',obj.RequestType);

                obj.ReqMsgStruct = varargin{1};
                indx = indx + 1;
            end

            nvPairs = struct('Timeout',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});
            callTimeout = coder.internal.getParameterValue(pStruct.Timeout,inf,varargin{indx:end});

            % Runtime verification for input timeout, return right away if
            % there is more than one output and input timeout is invalid.
            if callTimeout <= 0 && nargout > 1
                statusText = 'input';
                return;
            end

            % Address synatx: call(client,"Timeout",inf)
            % Since MATLAB Interpretation mode does not allow "0" as input
            % timeout, "0" will be passed to C++ class representing
            % infinite case.
            if isinf(callTimeout)
                callTimeout = 0;
            end

            % Compile time error check
            validateattributes(callTimeout,{'numeric'},...
                               {'scalar','nonempty','real','nonnegative'},'Call','Timeout');

            % Convert callTimout from s to ms
            if callTimeout < 0
                coder.internal.error('ros:mlros:serviceclient:CallWaitTimeout',sprintf('%.2f', double(callTimeout)));
            end
            callTimeoutMS = callTimeout * 1000;
            statusIndex = 2;
            coder.ceval('MATLABSvcClient_callService', obj.SvcClientHelperPtr, callTimeoutMS, coder.ref(statusIndex));

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
                coder.internal.error('ros:mlros:serviceclient:CallWaitTimeout',sprintf('%.2f', double(callTimeout)));
            else
                % Ignore warning here since this could be reached.
                % MATLAB cannot detect status update when it was passed as
                % a reference to an external cpp function.
                response = obj.ResponseMessage;
            end
        end

        function msg = rosmessage(obj)
        % ROSMESSAGE Create a new service request message
        %   MSG = ROSMESSAGE(CLIENT) creates and returns an empty message MSG.
        %   The message type of MSG is determined by the service that this
        %   CLIENT is connected to. The format of MSG is determined by the
        %   DataFormat of the service client. The message is the default
        %   request that you can use to call a service.

            msg = rosmessage(obj.RequestType,'DataFormat','struct');
        end

        function msg = get.RequestMessage(obj)
            coder.ceval('MATLABSvcClient_lock',obj.SvcClientHelperPtr);
            msg = obj.ReqMsgStruct;
            coder.ceval('MATLABSvcClient_unlock',obj.SvcClientHelperPtr);
        end

        function msg = get.ResponseMessage(obj)
            coder.ceval('MATLABSvcClient_lock',obj.SvcClientHelperPtr);
            msg = obj.RespMsgStruct;
            coder.ceval('MATLABSvcClient_unlock',obj.SvcClientHelperPtr);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'RequestType','ResponseType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS SvcClient';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_svcclient.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'ServiceType','ServiceName',...
                     'DataFormat','RequestType','ResponseType'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
