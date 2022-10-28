classdef Publisher < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
    % This class is for internal use only. It may be removed in the future.

    % Publisher - Code generation equivalent for ros.Publisher
    % See also ros.Publisher

    % Copyright 2020-2021 The MathWorks, Inc.
    %#codegen

    properties (SetAccess = private)
        %TopicName - The name of the publishing topic
        TopicName

        %MessageType - The message type of publishing messages
        MessageType

        %BufferSize - The buffer size of the outgoing queue
        BufferSize

        %IsLatching - Latch behavior when publishing message
        IsLatching

        %DataFormat - Message format required for use
        DataFormat
    end

    properties(Dependent, SetAccess = private)
        %NumSubscribers - The number of current subscribers
        NumSubscribers
    end

    properties (Access = private)
        MsgStruct
        PublisherHelper
    end

    methods
        function obj = Publisher(node, topic, varargin)
        %Publisher - Create a ROS publisher object
        %   Attach a new publisher to the ROS "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   publisher should publish. Please see the class documentation
        %   (help ros.Publisher) for more details.

            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');

            % Ensure varargin is not empty
            coder.internal.assert(nargin>2,'ros:mlroscpp:codegen:MissingMessageType',topic,'rospublisher');

            % Specialize Publisher class based on messageType
            coder.internal.prefer_const(varargin{1});

            if ~isempty(node)
                % A node cannot create another node in code generation
                coder.internal.assert(false, 'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Validate input topic
            topic = convertStringsToChars(topic);
            validateattributes(topic,{'char','string'},{'nonempty'}, ...
                               '','topic');

            % Define optional arguments
            opArgs.MessageType = @(d)coder.internal.isTextRow(d);
            % Define the parameter names
            NVPairNames = {'DataFormat','BufferSize','IsLatching'};
            % Select parsing options
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            % Parse the inputs
            pStruct = coder.internal.parseInputs(opArgs, NVPairNames,pOpts,varargin{:});
            % Retrive input values
            messageType = coder.internal.getParameterValue(pStruct.MessageType,'default',varargin{:});
            messageType = convertStringsToChars(messageType);
            validateattributes(messageType,{'char','string'},{'nonempty'}, ...
                               'Publisher','messageType');
            coder.internal.assert(contains(messageType,'/'),'ros:mlroscpp:codegen:MissingMessageType',topic,'rospublisher');

            % Retrive name-value pairs
            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'class',varargin{:});
            validateStringParameter(dataFormat,{'class','struct'},'Publisher','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'),...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','rospublisher');
            bufferSize = coder.internal.getParameterValue(pStruct.BufferSize,1,varargin{:});
            validateattributes(bufferSize,{'numeric'},...
                               {'scalar','nonempty','integer','nonnegative'},'Publisher','BufferSize');
            isLatching = coder.internal.getParameterValue(pStruct.IsLatching,true,varargin{:});
            validateattributes(isLatching,{'logical'},...
                               {'scalar','nonempty'},'Publisher','IsLatching');

            % Store input arguments
            obj.TopicName = topic;
            obj.MessageType = messageType;
            obj.BufferSize = bufferSize;
            obj.IsLatching = isLatching;
            obj.DataFormat = dataFormat;

            % Get and register code generation information
            cgInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,topic,messageType,'pub');
            msgStructGenFcn = str2func(cgInfo.MsgStructGen);
            obj.MsgStruct = msgStructGenFcn();  % Setup return type

            % Create an instance of MATLABPublisher object
            % template <class MsgType, class StructType>
            % MATLABPublisher(MsgType* msgPtr)
            coder.cinclude('mlroscpp_pub.h');
            templateTypeStr = ['MATLABPublisher<' cgInfo.MsgClass ',' cgInfo.MsgStructGen '_T>'];
            obj.PublisherHelper = coder.opaque(['std::unique_ptr<', templateTypeStr, '>'],...
                'HeaderFile','mlroscpp_pub.h');
            obj.PublisherHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                '>(new ', templateTypeStr, '());//']);
            coder.ceval('MATLABPUBLISHER_createPublisher',obj.PublisherHelper,...
                coder.rref(obj.TopicName), size(obj.TopicName,2), ...
                obj.BufferSize, obj.IsLatching)
        end

        function send(obj, msgToSend)
        % Ensure send is not optimized away
            coder.ceval('MATLABPUBLISHER_publish',obj.PublisherHelper,coder.rref(msgToSend));
        end

        function msgFromPub = rosmessage(obj)
        % ROSMESSAGE Create an empty message that you can send with this publisher
        %   MSG = ROSMESSAGE(PUB) creates and returns an empty message MSG. The
        %   message type of MSG is determined by the topic that the publisher
        %   PUB is advertising. The format of MSG is determined by the
        %   DataFormat of the publisher. The message MSG can be sent with this
        %   publisher.
        %
        %
        %   Example:
        %      % Create publisher and message
        %      chatPub = ros.Publisher(node,"/chatter","std_msgs/String");
        %      msgObj = ROSMESSAGE(chatPub);
        %
        %      % Improve performance by using struct messages
        %      posePub = ros.Publisher(node,"/pose","geometry_msgs/Pose2D","DataFormat","struct");
        %      msgStruct = ROSMESSAGE(posePub);
        %
        %   See also SEND.

            coder.inline('never');
            msgFromPub = rosmessage(obj.MessageType,'DataFormat','struct');
        end

        function numSubscribers = get.NumSubscribers(obj)
        %get.NumSubscribers - getter for NumSubscribers
            numSubscribers = 0;
            numSubscribers = coder.ceval('MATLABPUBLISHER_getNumSubscribers',obj.PublisherHelper);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'MessageType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS Publisher';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_pub.h',srcFolder);
            end
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
