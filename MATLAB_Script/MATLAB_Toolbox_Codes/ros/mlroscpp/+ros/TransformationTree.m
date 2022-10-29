classdef TransformationTree < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
%TransformationTree  Receive, send, and apply ROS transformations
%   ROS uses the tf2_ros transformation library to keep track of the relationship
%   between multiple coordinate frames. The relative transformations between
%   these coordinate frames are maintained in a tree structure. Querying
%   this tree lets you transform messages like poses and points between any two
%   coordinate frames.
%
%   The transformation tree changes over time and by default transformations
%   are buffered for up to 10 seconds. You can access transformations at
%   any time in this buffer window and the result will be interpolated to the
%   exact time you specify.
%
%   TFTREE = ros.TransformationTree(NODE) creates a ROS transformation
%   tree object TFTREE. NODE is the ros.Node object handle that
%   the transformation tree should attach to.
%   You can receive transformations and apply them to different stamped messages.
%   You can also send transformations to share them with the rest of the ROS network.
%
%   TFTREE = ros.TransformationTree(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify several
%   name-value pair arguments in any order as Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of ROS message to be expected by
%                     the transformation tree as inputs to transform and
%                     sendTransform, and returned from transform and
%                     getTransform.
%                     Using structs can be faster than using message objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   TransformationTree properties:
%      AvailableFrames - (Read-only) List of all available coordinate frames
%      LastUpdateTime  - (Read-only) Time when the last transform was received
%      BufferTime      - Time (in seconds) for which transformations are buffered
%      DataFormat      - (Read-Only) Message format required and returned
%
%   TransformationTree methods:
%      getTransform       - Return the transformation between two coordinate frames
%      canTransform       - Verify if transformation is available
%      transform          - Transform stamped messages into target coordinate frame
%      sendTransform      - Send a transform to the ROS network
%
%
%   Example:
%
%       % This example assumes that an existing ROS master is located
%       % at IP address 192.168.70.128 and that some ROS node publishes
%       % transformations between base_link and camera_depth_frame.
%       % For example, a real or simulated TurtleBot would do that.
%
%        % Create a node and connect to master
%        node = ros.Node("/testTf","192.168.70.128");
%
%        % Retrieve the transformation tree object
%        % Use struct message format for better performance
%        tftree = ros.TransformationTree(node,"DataFormat","struct")
%
%        % Buffer transformations for up to 15 seconds
%        tftree.BufferTime = 15
%
%        % Wait for the transform that takes data from "camera_depth_frame"
%        % to "base_link". This is blocking until the
%        % transformation is valid.
%        tform = getTransform(tftree,"base_link","camera_depth_frame","Timeout",Inf)
%
%        % Define a point [3 1.5 0.2] in the camera's coordinate frame
%        pt = rosmessage("geometry_msgs/PointStamped","DataFormat","struct");
%        pt.Header.FrameId = "camera_depth_frame";
%        pt.Point.X = 3;
%        pt.Point.Y = 1.5;
%        pt.Point.Z = 0.2;
%
%        % Transformation is available, so transform the point into the "base_link" frame
%        tfPt = transform(tftree,"base_link",pt)
%
%        % Display the transformed point coordinates
%        tfPt.Point
%
%        % Get the transformation that was valid 1 second ago. Wait for up to
%        % 2 seconds for the transformation to become available.
%        sourceTime = rostime("now","DataFormat","struct");
%        sourceTime.Sec = sourceTime.Sec - 1;
%        tform = getTransform(tftree,"base_link","camera_depth_frame",sourceTime,"Timeout",2)
%
%        % Apply the new transformation to the point
%        tfPt2 = rosApplyTransform(tform,pt)
%
%   See also ROSTF.

%   Copyright 2014-2021 The MathWorks, Inc.

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

    properties (Dependent)
        %BufferTime - Time (in seconds) for which transformations are buffered
        %   If you change the buffer time from its current value,
        %   the transformation tree and all transformations are
        %   re-initialized.
        %   By default, the buffer length is 10 seconds.
        BufferTime
    end

    properties (Constant, Access = ?matlab.unittest.TestCase)
        %DefaultBufferTime - The default buffer time (in seconds)
        DefaultBufferTime = 10
    end

    properties (Access = private)
        %TfTopicType - The message type of the /tf topic
        TfTopicType
    end

    properties (Access = ?matlab.unittest.TestCase)
        %Parser - Helper object for parsing tasks
        Parser
    end

    properties (Transient, Access = ?matlab.unittest.TestCase)
        %InternalNode - Internal representation of the node object
        %   Node required to attach the tfTree
        InternalNode = []

        %TfHandle - TransformationHandle , Internal representation of
        %   TfTree. Required to call appropriate backend APIs
        TfHandle = []

        %HasIssuedGetTformWarning - Indicates if this object already issued behavior warning
        %   The behavior of getTransform will change in a future release.
        HasIssuedGetTformWarning = false
    end

    methods
        function obj = TransformationTree(node, varargin)
        %TransformationTree Construct a transformation tree object
        %   Please see the class documentation
        %   (help ros.TransformationTree) for more details.

            import ros.internal.Settings;

            narginchk(1,3)

            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Validate that node is a ros.Node object
            validateattributes(node, {'ros.Node'}, {'scalar', 'nonempty'}, ...
                               'TransformationTree', 'node', 1);

            % Create parser helper instance and determine data format
            obj.Parser = ros.internal.tf.TransformationTreeParser(varargin{:});
            setDataFormat(obj, obj.Parser.DataFormat)

            % If something goes wrong in internal call, appropriate error is
            % thrown from back-end.
            returnCall = createTfTree(node.InternalNode, node.ServerNodeHandle);

            if isempty(returnCall) || ~isstruct(returnCall)
                error(message('ros:mlroscpp:node:InvalidReturnCallError'))
            elseif ~isfield(returnCall, 'handle') || ...
                    isempty(returnCall.handle)
                error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
            end
            obj.TfHandle = returnCall.handle;
            obj.InternalNode = node.InternalNode;

            % Get topic type of /tf. Default to tf2 if topic does not exist.
            obj.TfTopicType = obj.retrieveTopicType(node, Settings.Tf2MessageType);
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);
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

            narginchk(3,6);
            if nargin == 3
                % For backwards compatibility, return empty on error if the
                % user only specifies target and source frame. Use the old
                % parsing code as well.

                [targetFrame, sourceFrame] = obj.Parser.parseGetTransformBackwardsCompatibleInput(targetFrame, sourceFrame);
                tform = obj.getTransformBackwardsCompatible(targetFrame, sourceFrame);
            else
                % New behavior. Display errors if the input is invalid or
                % if the transformation cannot be found. This behavior will
                % be the default in the future.

                defaults = struct(...
                    'Timeout', 0, ...
                    'SourceTime', 0);
                [targetFrame, sourceFrame, sourceTime, timeout] = ...
                    obj.Parser.parseGetTransformInput(defaults, targetFrame, sourceFrame, varargin{:});

                errorCode = 0;
                try
                    % Wait until the end of the timeout
                    util = ros.internal.Util.getInstance;
                    util.waitUntilTrue( @() canTransform(obj, targetFrame, sourceFrame, ...
                                                         sourceTime), timeout );
                catch ex
                    if ~strcmp(ex.identifier, 'ros:mlros:util:WaitTimeout')
                        % Rethrow exception if an unexpected exception is seen
                        rethrow(ex);
                    end
                end

                try
                    % If Transform is available, no need to wait again. So
                    % the timeout sec = 0 and nsec = 0 sent to backend.
                    tform = lookupTransform(obj.InternalNode, obj.TfHandle, ...
                                            targetFrame, sourceFrame, sourceTime.Sec, sourceTime.Nsec, 0, 0);
                catch ex
                    if isequal(ex.identifier, 'ros:internal:transport:TFConnectivityException')
                        errorCode = 1;
                    elseif isequal(ex.identifier, 'ros:internal:transport:TFExtrapolationException')
                        errorCode = 2;
                    elseif isequal(ex.identifier, 'ros:internal:transport:TFLookupException')
                        errorCode = 3;
                    elseif isequal(ex.identifier, 'ros:internal:transport:TFInvalidArgumentException')
                        errorCode = 4;
                    end
                end

                % Handle error accordingly
                obj.handleErrorCode(errorCode,targetFrame,sourceFrame,sourceTime);
            end

            % Convert the transform struct to class if necessary
            if obj.UseObjectMsg
                tform = ros.msg.geometry_msgs.TransformStamped(tform);
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

            narginchk(3,4);

            defaultSourceTime = 0;
            [targetFrame, sourceFrame, sourceTime] = ...
                obj.Parser.parseCanTransformInput(defaultSourceTime, targetFrame, sourceFrame, varargin{:});

            % Appropriate error is being thrown from backend in case of
            % failure of the MCOS function call.
            returnCall = canTransform(obj.InternalNode, obj.TfHandle, ...
                                      targetFrame, sourceFrame, sourceTime.Sec, sourceTime.Nsec, 0, 0);

            if isempty(returnCall) || ~isstruct(returnCall) || ...
                    ~isfield(returnCall, 'res') || isempty(returnCall.res)
                error(message('ros:mlroscpp:node:InvalidReturnCallError'))
            end

            isAvailable = returnCall.res;
        end

        function sendTransform(obj, tf)
        %sendTransform - Send a transform to the ROS network
        %   sendTransform(OBJ,TF) broadcasts a transform TF to the
        %   ROS network. TF is a scalar message or a message list of type
        %   geometry_msgs/TransformStamped. The format of the message must
        %   match the DataFormat of the TransformationTree.

        % Accept vector input and validate first message for format/type
            validateattributes(tf, {'ros.msggen.geometry_msgs.TransformStamped', 'struct'}, ...
                               {'vector', 'nonempty'}, ...
                               'sendTransform', 'tf');
            obj.validateInputMessage(tf(1), 'geometry_msgs/TransformStamped', ...
                                     'TransformationTree', 'sendTransform')

            % Convert the class to struct if necessary
            if obj.UseObjectMsg
                tf = toROSStruct(tf);
            end

            % Check for invalid-quaternation
            for i = 1:numel(tf)
                tf(i) = handleInvalidQuaternation(obj, tf(i));
            end

            % Back-end takes care of both scalar case and a list of messages.
            sendTransform(obj.InternalNode, obj.TfHandle, tf);
        end

        function waitForTransform(obj, targetFrame, sourceFrame, varargin)
        %waitForTransform Block until a transformation is available
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

        % By default, block until transformation is available
            defaults.timeout = Inf;
            [targetFrame, sourceFrame, timeout] = obj.Parser.parseWaitArguments('waitForTransform', defaults, ...
                                                                                targetFrame, sourceFrame, varargin{:});

            % Wait until transformation is available, or timeout occurs
            try
                sourceTime = rostime(0);
                util = ros.internal.Util.getInstance;
                util.waitUntilTrue( @() canTransform(obj, targetFrame, sourceFrame, ...
                                                     sourceTime), timeout );
            catch
                error(message('ros:mlros:tf:WaitTimeout', num2str(timeout)));
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

            defaultSourceTime = 0;
            [targetFrame, sourceFrame, msg, sourceTime] = ...
                obj.Parser.parseTransformInput(defaultSourceTime, targetFrame, msg, varargin{:});

            % Lookup the transformation based on the given target frame and
            % the source frame in the message.
            tf = obj.getTransform(targetFrame, sourceFrame, sourceTime);

            % We have a valid transformation. Apply it to the input.
            th = ros.internal.TransformHelper(tf);
            tfEntity = th.transform(msg);
        end
    end

    methods
        function frameNames = get.AvailableFrames(obj)
        %get.AvailableFrames Retrieve all frame names in the tree

        % Retrieve all names from ROS node
            frames = availableFrames(obj.InternalNode, obj.TfHandle)';

            % Convert list to MATLAB cell array of strings and
            % remove all duplicates. The resulting list is sorted.
            frameNames = unique(frames);
        end

        function updateTime = get.LastUpdateTime(obj)
        %get.LastUpdateTime Retrieve last time the tree was updated

            time = lastUpdateTime(obj.InternalNode, obj.TfHandle);

            if isempty(time) || (isequal(time.sec, 0) && isequal(time.nsec, 0))
                updateTime = struct.empty(0, 1);
                if obj.UseObjectMsg
                    updateTime = ros.msg.Time(updateTime);
                end
            else
                updateTime = rostime(time.sec, time.nsec, ...
                                     'DataFormat', obj.DataFormat);
            end
        end

        function bufferTime = get.BufferTime(obj)
        %get.LastUpdateTime Retrieve the current length of the buffer

            cacheTime = getCacheTime(obj.InternalNode, obj.TfHandle);
            bufferTime = double(cacheTime.sec)+1e-9*double(cacheTime.nsec);
        end

        function set.BufferTime(obj, bufferTime)
        %set.BufferTime Set the length of the buffer

            validBufferTime = obj.Parser.parseBufferTime(bufferTime);
            bufferRosTime = rostime(validBufferTime);

            % Only re-initialize transformation tree if requested time is
            % different from the current buffer time
            currentBufferTime = getCacheTime(obj.InternalNode, obj.TfHandle);
            currentRosTime = rostime(currentBufferTime.sec,currentBufferTime.nsec);

            if bufferRosTime ~= currentRosTime
                setCacheTime(obj.InternalNode, obj.TfHandle, ...
                             bufferRosTime.Sec, bufferRosTime.Nsec);
            end
        end
    end

    methods (Static, Access = ?matlab.unittest.TestCase)
        function tfType = retrieveTopicType(node, defaultType)
        %retrieveTopicType Retrieve and return the message type of tf topic
        %   There are two different versions of ROS's tf system. Both
        %   tf and tf2 publish their data on the "/tf" topic, but one
        %   has message type tf/tfMessage and the other tf2_msgs/TFMessage.
        %
        %   This function determines what message type the /tf topic
        %   has and returns it in TFTYPE. If the tf topic does not exist, the
        %   default message type DEFAULTTYPE is returned.

            import ros.internal.Settings;

            tfType = defaultType;

            try
                tfType = ros.internal.NetworkIntrospection.getPublishedTopicType(...
                    Settings.TfTopic, false, node.MasterURI);
            catch
                % No need to throw an exception here. Just return the
                % default
                return;
            end

            % Check tfType for validity
            if ~ismember(tfType, {Settings.TfMessageType, Settings.Tf2MessageType})
                error(message('ros:mlros:tf:TfTypeNotValid', tfType, ...
                              [Settings.TfMessageType ', ' Settings.Tf2MessageType]));
            end
        end
    end

    methods (Access = private)
        function handleErrorCode(obj, errorCode, targetFrame, sourceFrame, sourceTime)
        %handleErrorCode Handle the error code returned by lookupTransform
        %   Display an appropriate error message.

            switch errorCode
              case 0
                % errorCode 0 is not an error condition
                return;
              case 1
                % errorCode 1 means the frames are not connected in the
                % tree. They are in unconnected sub-graphs of the tree.
                error(message('ros:mlros:tf:FramesNotConnected', targetFrame, sourceFrame));
              case 2
                % errorCode 2 means the transformation is outside the current buffer window
                % The time could be in the future or it could be
                % further in the past than the start of the buffer
                % window
                error(message('ros:mlros:tf:SourceTimeOutsideBuffer', targetFrame, sourceFrame, ...
                              sprintf('%d:%d', sourceTime.Sec, sourceTime.Nsec)));
              case 3
                % errorCode 3 means that one (or both) of the frames is not in the buffer
                % Give the user a concrete error message which case
                % applies.
                frameName = '';
                isTargetValid = ismember(targetFrame,obj.AvailableFrames);
                isSourceValid = ismember(sourceFrame,obj.AvailableFrames);

                if ~isTargetValid && ~isSourceValid
                    % Both frame names are invalid
                    error(message('ros:mlros:tf:BothFrameNamesNotFound', targetFrame, sourceFrame));
                end

                % Only one frame name is invalid. Find out which one.
                if ~isTargetValid
                    frameName = targetFrame;
                elseif ~isSourceValid
                    frameName = sourceFrame;
                end

                if isempty(frameName)
                    % Frames exist but not connected.
                    obj.handleErrorCode(1,targetFrame,sourceFrame,sourceTime);
                else
                    error(message('ros:mlros:tf:FrameNameNotFound', frameName));
                end

              otherwise
                % errorCode 4 means some other error occurred
                error(message('ros:mlros:tf:NoValidTransformation', targetFrame, sourceFrame));
            end
        end

        function tf = handleInvalidQuaternation(~, tf)
        % Check if a rotation quaternion in a transformation is invalid.
        % A quaternion is invalid if its L2 norm is 0 or very close to
        % zero. Taking the inverse of such a quaternion leads to invalid
        % results. In that case, use the unit quaternion
        %See g1228205.

            normThresh = 1e-7;
            rot = tf.Transform.Rotation;
            quatNorm =  sqrt(rot.X^2 + rot.Y^2 + rot.Z^2 + rot.W^2);
            if quatNorm < normThresh
                rot.X = 0;
                rot.Y = 0;
                rot.Z = 0;
                rot.W = 1;
                tf.Transform.Rotation = rot;

                % display warning
                warning(message('ros:mlros:tf:InvalidQuaternion', ...
                                tf.Header.FrameId, tf.ChildFrameId));
            end
        end

        function tform = getTransformBackwardsCompatible(obj, targetFrame, sourceFrame)
        %getTransformBackwardsCompatible Ensure backwards-compatible getTransform behavior
        %   If only TARGETFRAME and SOURCEFRAME are specified and the
        %   transformation is not available or the frame names do not
        %   exist, getTransform returns an empty transformation. In the
        %   future, this condition will trigger errors.


            try
                tform = lookupTransform(obj.InternalNode, obj.TfHandle, ...
                                        targetFrame, sourceFrame, 0, 0, 0, 0);
            catch
                tform = [];
                if ~obj.HasIssuedGetTformWarning
                    % Warn the user once per object about the future
                    % behavior change
                    warning(message('ros:mlros:tf:GetTransformBehaviorChange'));
                    obj.HasIssuedGetTformWarning = true;
                end
            end
        end
    end

    methods (Access = {?robotics.core.internal.mixin.Unsaveable, ?matlab.unittest.TestCase})
        function delete(obj)
        %DELETE Delete the transformation tree object
        %   Please note that this function is private to avoid explicit
        %   invocations.

            try
                % Only try to shut down if tree created
                if ~isempty(obj.InternalNode) && ~isempty(obj.TfHandle)
                    deleteTfTree(obj.InternalNode, obj.TfHandle);
                    obj.InternalNode = [];
                end
            catch
                warning(message('ros:mlros:tf:ShutdownError'));
            end
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.TransformationTree';
        end
    end
end
