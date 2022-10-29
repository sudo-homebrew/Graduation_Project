classdef TransformationTree < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
% This class is for internal use only. It may be removed in the future.
%
% TransformationTree - Code generation equivalent for
% ros.TransformationTree
%
% See also ros.TransformationTree
%
%#codegen

%   Copyright 2021 The MathWorks, Inc.

    properties (Access = private)
        %TransformHelper - helper handle for MATLABROSTransform
        TransformHelper
    end

    properties (Dependent)
        %BufferTime - Time (in seconds) for which transformations are buffered
        %   If you change the buffer time from its current value, the
        %   transformation tree and all transformations are re-initialized.
        %   By default, the buffer length is 10 seconds.
        BufferTime
    end

    properties (Dependent, SetAccess = private)
        %AvailableFrames - List of all available coordinate frames
        %   The list of all coordinate frames in the transformation tree
        %   is returned as a cell array of strings. It is empty
        %   if no frames are in the tree.
        AvailableFrames

        %LastUpdateTime - Time when the last transform was received
        %   The time is returned as a ros.msg.Time object. It
        %   is empty if no transforms have been received yet.
        LastUpdateTime
    end

    properties (SetAccess = immutable)
        %DataFormat - Message format required and returned
        DataFormat
    end

    properties (Constant)
        %DefaultBufferTime - The default buffer time (in seconds)
        DefaultBufferTime = 10
        %DefaultSourceTime - The default source time (in seconds)
        DefaultSourceTime = 0
    end

    methods
        function obj = TransformationTree(node, varargin)
        %TransformationTree Construct a transformation tree object
        %   Please see the class documentation (help
        %   ros.TransformationTree) for more details.

        % Check input arguments
            if ~isempty(node)
                % A node cannot create another node in codegen
                coder.internal.assert(false, 'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Parse NV pairs
            nvPairs = struct('DataFormat',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{:});
            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{:});
            validateStringParameter(dataFormat,{'object','struct'},'TransformationTree','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'),...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','TransformationTree');
            obj.DataFormat = dataFormat;

            % Create an instance of MATLABROSTransform object and store
            % handle to TransformHelper
            obj.TransformHelper = coder.opaque('MATLABROSTransform','HeaderFile','mlroscpp_transform.h');
            obj.TransformHelper = coder.ceval('MATLABROSTransform');
            coder.ceval('UNUSED_PARAM',obj.TransformHelper);

            % Trigger generation of ros_structmsg_conversion.h file
            % (g2585525)
            rosmessage('geometry_msgs/TransformStamped','DataFormat','struct');
        end

        function tform = getTransform(obj, targetFrame, sourceFrame, varargin)
        %getTransform Return the transformation between two coordinate frames
        %   TF = getTransform(OBJ,TARGETFRAME,SOURCEFRAME) gets and
        %   returns the latest known transformation between two coordinate frames.
        %   TF represents the transformation that takes coordinates
        %   in the SOURCEFRAME into the corresponding coordinates in
        %   the TARGETFRAME. The return TF is empty if this transformation does not
        %   exist in the tree.
        %
        %   TF = getTransform(OBJ,TARGETFRAME,SOURCEFRAME,SOURCETIME)
        %   returns the transformation at the time SOURCETIME. An error
        %   is displayed if the transformation at that time is not
        %   available.
        %
        %   TF = getTransform(___,Name,Value) provides additional options
        %   specified by one or more Name,Value pair arguments. You can
        %   specify several name-value pair arguments in any order as
        %   Name1,Value1,...,NameN,ValueN:
        %
        %      "Timeout"  -  Specifies a timeout period, in seconds.
        %                    If the transformation does not become
        %                    available within the specified period,
        %                    getTransform displays an error message.
        %                    The default value is 0, so that
        %                    getTransform does not wait at all, but
        %                    checks if the transformation is available
        %                    instantly.
        %
        %   You can transform messages like point clouds and
        %   vectors directly by calling the "transform" function.
        %
        %   The behavior of getTransform will change in a future
        %   release. If only TARGETFRAME and SOURCEFRAME are specified
        %   and the transformation is not available, getTransform will
        %   display an error and not return an empty TF. To check if a
        %   transformation is available, use canTransform.
        %
        %
        %   Example:
        %
        %        % Create the transformation tree object
        %        tftree = rostf
        %
        %        % Wait for the transform that takes data from "base_link"
        %        % to "odom". This calls is blocking until the transformation is available.
        %        tform = getTransform(tftree,"odom","base_link","Timeout",Inf)
        %
        %        % Get the transformation that was valid 3 second ago.
        %        tform = getTransform(tftree,"odom","base_link",rostime("now") - 3)
        %
        %        % Wait for a transformation that is valid a few seconds in the future.
        %        % Use a timeout to wait until the transformation becomes available.
        %        tform = getTransform(tftree,"odom","base_link",rostime("now") + 1,"Timeout",5)
        %
        %   See also canTransform, TRANSFORM.

            coder.inline('never');
            % Validate input arguments
            coder.internal.narginchk(3,6,nargin);
            targetFrame = convertStringsToChars(targetFrame);
            sourceFrame = convertStringsToChars(sourceFrame);
            validateattributes(targetFrame, {'char','string'},{'nonempty'},...
                               'canTransform','targetFrame');
            validateattributes(sourceFrame, {'char','string'},{'nonempty'},...
                               'canTransform','sourceFrame');

            % Default sourceTime and timeout are both 0
            defaultTime = 0;
            if isempty(varargin)
                % Syntax: getTransform('TARGETFRAME','SOURCEFRAME')
                sourceTime = rostime(defaultTime,'DataFormat','struct');
            elseif isnumeric(varargin{1})
                % Syntax:
                % getTransform('TARGETFRAME','SOURCEFRAME',SOURCETIME) or
                % getTransform('TARGETFRAME','SOURCEFRAME',SOURCETIME,'Timeout',timeout)
                sourceTime = ros.internal.Parsing.validateROSTimeStruct(varargin{1},'getTransform','sourceTime');
                indx = 2;
            else
                % Syntax:
                % getTransform('TARGETFRAME','SOURCEFRAME','Timeout',timeout)
                sourceTime = rostime(defaultTime,'DataFormat','struct');
                indx = 1;
            end

            timeout = 0;
            if nargin>4
                % Retrieve name-value pairs
                % Can only be either
                % getTransform('TARGETFRAME','SOURCEFRAME','Timeout',timeout)
                % or
                % getTransform('TARGETFRAME','SOURCEFRAME',SOURCETIME,'Timeout',timeout)
                nvPairs = struct('Timeout',uint32(0));
                pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
                pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});
                timeout = coder.internal.getParameterValue(pStruct.Timeout,defaultTime,varargin{indx:end});
                validateattributes(timeout,{'numeric'},...
                                   {'scalar','nonempty','positive'},'getTransform','Timeout');
            end

            if isfinite(timeout)
                timeoutStruct = rostime(timeout,'DataFormat','struct');
            else
                % Address syntax:
                % getTransform('TARGETFRAME','SOURCEFRAME','Timeout',inf)
                % Wait until transformation is available
                while(~canTransform(obj,targetFrame,sourceFrame,sourceTime))
                    % Avoid optimizing away
                    coder.ceval('//',targetFrame);
                end
                timeoutStruct = rostime(defaultTime,'DataFormat','struct');
            end

            tform = rosmessage("geometry_msgs/TransformStamped","DataFormat","struct");
            coder.cinclude('<functional>');
            coder.ceval('std::mem_fn(&MATLABROSTransform::lookupTransform)', ...
                        coder.ref(obj.TransformHelper), coder.wref(tform),coder.rref(targetFrame), size(targetFrame,2),...
                        coder.rref(sourceFrame), size(sourceFrame, 2), ...
                        sourceTime.Sec, sourceTime.Nsec, timeoutStruct.Sec, timeoutStruct.Nsec);
        end

        function sendTransform(obj, tf)
        %sendTransform - Send a transform to the ROS network
        %   sendTransform(OBJ,TF) broadcasts a transform TF to the
        %   ROS network. TF is a scalar message or a message list of type
        %   geometry_msgs/TransformStamped. The format of the message must
        %   match the DataFormat of the TransformationTree.

            validateattributes(tf,'struct',{'vector','nonempty'},'sendTransform','tf');

            if ~isequal(tf(1).MessageType, 'geometry_msgs/TransformStamped')
                coder.internal.error('ros:mlroscpp:message:InputTypeMismatch','geometry_msgs/TransformStamped');
            end

            coder.cinclude('<functional>');
            for i = 1:numel(tf)
                % send tf to ROS network
                coder.ceval('std::mem_fn(&MATLABROSTransform::sendTransform)', ...
                            coder.ref(obj.TransformHelper), tf(i));
            end

        end

        function isAvailable = canTransform(obj, targetFrame, sourceFrame, varargin)
        %canTransform Verify if transformation is available
        %   ISAVAILABLE = canTransform(OBJ,TARGETFRAME,SOURCEFRAME)
        %   verifies if a transformation that takes coordinates
        %   in the SOURCEFRAME into the corresponding coordinates in
        %   the TARGETFRAME is available. ISAVAILABLE is TRUE if that
        %   transformation is available and FALSE otherwise.
        %   Use getTransform to retrieve the transformation.
        %
        %   ISAVAILABLE = canTransform(OBJ,TARGETFRAME,SOURCEFRAME,SOURCETIME)
        %   verifies that the transformation is available for the time
        %   SOURCETIME. If SOURCETIME is outside of the buffer window
        %   for the transformation tree, the function returns FALSE.
        %   Use getTransform with the SOURCETIME argument to retrieve
        %   the transformation.
        %
        %   See also getTransform.

            coder.inline('never');
            coder.cinclude('<functional>');
            % Validate input arguments
            coder.internal.narginchk(3,4,nargin);
            targetFrame = convertStringsToChars(targetFrame);
            sourceFrame = convertStringsToChars(sourceFrame);
            validateattributes(targetFrame, {'char','string'},{'nonempty'},...
                               'canTransform','targetFrame');
            validateattributes(sourceFrame, {'char','string'},{'nonempty'},...
                               'canTransform','sourceFrame');

            if isempty(varargin)
                % Syntax: canTransfrom('TARGETFRAME','SOURCEFRAME')
                sourceTime = rostime(obj.DefaultSourceTime,'DataFormat','struct');
            else
                % Syntax: canTransfrom('TARGETFRAME','SOURCEFRAME',SOURCETIME)
                % return FALSE if SOURCETIME is outside of the buffer
                % window
                sourceTime = ros.internal.Parsing.validateROSTimeStruct(varargin{1},'canTransform','sourceTime');
                if ((sourceTime.Sec + 1e-9*sourceTime.Nsec) > obj.DefaultBufferTime)
                    isAvailable =  false;
                    return;
                end
            end

            % Note: there is no failure here since there is no backend MCOS
            % function call as what we did in MATLAB interpretation.
            % When passing to coder.ceval as input argument, a new variable
            % need to be created so that there is no string/char conflict.
            targetFrameRef = targetFrame;
            sourceFrameRef = sourceFrame;
            isAvailable = false;
            isAvailable = coder.ceval('std::mem_fn(&MATLABROSTransform::canTransform)', ...
                                      coder.ref(obj.TransformHelper), coder.rref(targetFrameRef), size(targetFrameRef,2),...
                                      coder.rref(sourceFrameRef), size(sourceFrameRef, 2), ...
                                      sourceTime.Sec, sourceTime.Nsec);
        end

        function waitForTransform(obj, targetFrame, sourceFrame, varargin)
        %waitForTransform - Block until a transformation is available
        %   waitForTransform will be removed in a future release.
        %   Use getTransform with a timeout instead.
        %
        %   waitForTransform(OBJ,TARGETFRAME,SOURCEFRAME) blocks
        %   MATLAB from running the current program until the
        %   transformation that takes data from the SOURCEFRAME into
        %   the TARGETFRAME is available in the transformation tree OBJ.
        %
        %   waitForTransform(OBJ,TARGETFRAME,SOURCEFRAME,TIMEOUT)
        %   specifies a TIMEOUT period, in seconds. If the
        %   transformation does not become available
        %   and the timeout period elapses, waitForTransform displays an error
        %   message and lets MATLAB continue running the current program.
        %
        %   To unblock waitForTransform and let MATLAB continue running
        %   the program, you can press Ctrl+C at any time.
        %
        %   See also getTransform.

            coder.inline('never');
            % Validate input arguments
            coder.internal.narginchk(3,4,nargin);
            targetFrame = convertStringsToChars(targetFrame);
            sourceFrame = convertStringsToChars(sourceFrame);
            validateattributes(targetFrame, {'char','string'},{'nonempty'},...
                               'waitForTransform','targetFrame');
            validateattributes(sourceFrame, {'char','string'},{'nonempty'},...
                               'waitForTransform','sourceFrame');

            coder.cinclude('<functional>');
            if isempty(varargin) || isinf(varargin{1})
                % Wait until transformation is available
                while(~canTransform(obj,targetFrame,sourceFrame))
                    % Avoid optimizing away
                    coder.ceval('//',targetFrame);
                end
            else
                % User specified timeout
                % Compile time error check
                validateattributes(varargin{1},{'numeric'},...
                                   {'scalar','nonempty','real','nonnegative'},'waitForTransform','Timeout');
                timeout = varargin{1};

                timeStatus = false;
                isAvailable = false;

                startTime = rostime("now","DataFormat","struct");
                while ~(timeStatus || isAvailable)
                    currentTime = rostime("now","DataFormat","struct");
                    isAvailable = canTransform(obj,targetFrame,sourceFrame);
                    if ge((currentTime.Sec - startTime.Sec) + 1e-9*(currentTime.Nsec - startTime.Nsec), timeout)
                        timeStatus = true;
                    end
                end
            end
        end

        function tfEntity = transform(obj, targetFrame, msg, varargin)
        %TRANSFORM Transform stamped messages into target coordinate frame
        %   TFMSG = TRANSFORM(OBJ,TARGETFRAME,MSG) retrieves the
        %   latest transformation that takes data from the MSG's coordinate
        %   frame to the TARGETFRAME. The transformation is then applied
        %   to the data in MSG. MSG is a ROS message of a specific type
        %   and the transformed message is returned in TFMSG. An error
        %   is displayed if the transformation does not exist.
        %
        %   TFMSG = TRANSFORM(OBJ,TARGETFRAME,MSG,"msgtime")
        %   uses the timestamp in the header of MSG as source time
        %   to retrieve and apply the transformation.
        %
        %   TFMSG = TRANSFORM(OBJ,TARGETFRAME,MSG,SOURCETIME)
        %   uses the time SOURCETIME to retrieve and apply the
        %   transformation to the MSG.
        %
        %   This function determines the type of the input message
        %   MSG and apply the appropriate transformation method. If a
        %   particular message type cannot be handled by this object,
        %   an error is displayed.
        %
        %   Supported message types include:
        %    - geometry_msgs/QuaternionStamped
        %    - geometry_msgs/Vector3Stamped
        %    - geometry_msgs/PointStamped
        %    - geometry_msgs/PoseStamped
        %    - sensor_msgs/PointCloud2
        %
        %
        %   Example:
        %
        %      % Define a point [3 1.5 0.2] in the camera's coordinate frame
        %      pt = rosmessage("geometry_msgs/PointStamped");
        %      pt.Header.FrameId = 'camera_depth_frame';
        %      pt.Point.X = 3;
        %      pt.Point.Y = 1.5;
        %      pt.Point.Z = 0.2;
        %
        %      % Transform the point into the "base_link" frame
        %      % This assumes that the transformation between "base_link"
        %      % and "camera_depth_frame" is available.
        %      tfPt = transform(tftree,"base_link",pt)
        %
        %      % You can also transform an unstamped point by wrapping
        %      % it into a stamped message
        %      ptUnstamped = rosmessage("geometry_msgs/Point");
        %      ptUnstamped.Point.X = -7.8;
        %      pt.Point = ptUnstamped;
        %
        %      tfPtUnstamped = transform(tftree,"base_link",pt)
        %
        %   See also getTransform.

            coder.inline('never');
            % Validate input arguments
            % message type validation will be handled by rosApplyTransform
            coder.internal.narginchk(3,4,nargin);
            targetFrame = convertStringsToChars(targetFrame);
            validateattributes(targetFrame, {'char','string'},{'nonempty'},...
                               'transform','targetFrame');

            % Extract source time
            if isempty(varargin)
                % Syntax: transform(OBJ,TARGETFRAME,MSG)
                sourceTime = obj.DefaultSourceTime;
            elseif isequal(varargin{1},"msgtime")
                % Syntax: transform(OBJ,TARGETFRAME,MSG,"msgtime")
                sourceTime = msg.Header.Stamp.Sec + 1e-9*msg.Header.Stamp.Nsec;
            else
                % Syntax: transform(OBJ,TARGETFRAME,MSG,SOURCETIME)
                % User specified timeout
                validateattributes(varargin{1},{'numeric'},...
                                   {'scalar','nonempty','real','nonnegative'},'transform','SOURCETIME');
                sourceTime = varargin{1};
            end

            % Lookup the transformation based on the given target frame and
            % the source frame in the messsage.
            sourceFrame = msg.Header.FrameId;
            tf = obj.getTransform(targetFrame,sourceFrame,sourceTime);

            % We have a valid transformation. Apply it to the input.
            tfEntity = rosApplyTransform(tf, msg);
        end

        function bufferTime = get.BufferTime(obj)
        %get.BufferTime - getter for BufferTime

            coder.inline('never');
            coder.cinclude('<functional>');
            %emptyCacheTime = rostime(0,'DataFormat','struct');
            %currentSec = emptyCacheTime.Sec;
            %currentNsec = emptyCacheTime.Nsec;
            currentSec = uint32(0);
            currentNsec = uint32(0);
            cacheTimeStatus = false;
            cacheTimeStatus = coder.ceval('std::mem_fn(&MATLABROSTransform::getCacheTime)', coder.ref(obj.TransformHelper), coder.ref(currentSec),coder.ref(currentNsec));

            if(cacheTimeStatus)
                cacheTime = rostime(currentSec,currentNsec,'DataFormat','struct');
            else
                cacheTime = rostime(obj.DefaultBufferTime,0,'DataFormat','struct');
            end
            bufferTime = double(cacheTime.Sec) + 1e-9*double(cacheTime.Nsec);
        end

        function set.BufferTime(obj, bufferTime)
        %set.BufferTime - setter for BufferTime

            coder.inline('never');
            coder.cinclude('<functional>');
            % Validate valid bufferTime
            validateattributes(bufferTime, {'numeric'}, {'nonempty', 'scalar', 'real', 'positive', ...
                                                         'nonnan', 'finite', '<', ros.internal.Parsing.MaxTimeNumeric}, ...
                               'TransformationTree', 'BufferTime');
            validBufferTime = double(bufferTime);
            bufferRosTime = rostime(validBufferTime,'DataFormat','struct');

            % Only re-initialize transformation tree if requrested time is
            % different from the current buffer time
            currentRosTime = rostime(obj.BufferTime,'DataFormat','struct');

            % Set cache time if bufferRosTime is not equal to
            % currentRosTime
            if ~isequal(bufferRosTime,currentRosTime)
                coder.ceval('std::mem_fn(&MATLABROSTransform::setCacheTime)', coder.ref(obj.TransformHelper), bufferRosTime.Sec, bufferRosTime.Nsec);
            end
        end

        function frameNames = get.AvailableFrames(obj)
        %get.AvailableFrames Retrieve all frame names in the tree

        % Retrieve all names from ROS node

            numberOfFrames = int32(0);
            coder.cinclude('<functional>');
            numberOfFrames = coder.ceval('std::mem_fn(&MATLABROSTransform::updateAndGetNumOfFrames)', coder.ref(obj.TransformHelper));
            frameNames = cell(numberOfFrames,1);

            for key=1:numel(frameNames)
                index = int32(key-1);
                frameNameLength = int32(0);
                frameNameLength = coder.ceval('std::mem_fn(&MATLABROSTransform::getFrameNameLength)', coder.ref(obj.TransformHelper),index);
                frameEntry = char(zeros(1,frameNameLength));
                coder.ceval('std::mem_fn(&MATLABROSTransform::getAvailableFrame)', coder.ref(obj.TransformHelper),index,coder.wref(frameEntry));
                frameNames{key} = frameEntry;
            end
        end

        function updateTime = get.LastUpdateTime(obj)
        %get.LastUpdateTime Retrieve last time the tree was updated

            coder.inline('never');
            doNotOptimize(obj);
            coder.internal.assert(false, 'ros:mlroscpp:codegen:UnsupportedMethodCodegen', ...
                                  'get.LastUpdateTime');
            updateTime = rostime(0,"DataFormat","struct");
        end
    end

    methods (Static)
        function ret = getDescriptiveName(~)
            ret = 'ROS TransformationTree';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_transform.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'DataFormat'};
        end
    end

    methods (Access = private)
        function doNotOptimize(obj)
        %DONOTOPTIMIZE - avoid optimizing away codes during Code Generation
            coder.ceval('//',coder.wref(obj.TransformHelper));
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
