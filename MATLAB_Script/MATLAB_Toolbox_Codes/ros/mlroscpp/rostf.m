function tftree = rostf(varargin)
%ROSTF Receive, send, and apply ROS transformations
%   TFTREE = ROSTF creates a ROS transformation tree object TFTREE. The
%   object allows you to access the tf coordinate transformations that
%   are shared on the ROS network. You can receive transformations and
%   apply them to stamped ROS messages. You can also send transformations
%   and share them with the rest of the ROS network.
%
%   TFTREE = ROSTF(___,Name,Value) provides additional options specified by
%   one or more Name,Value pair arguments. You can specify several
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
%   ROS uses the tf transform library to keep track of the relationship
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
%
%   Example:
%
%       % This example assumes that an existing ROS master is located
%       % at IP address 192.168.70.128 and that some ROS node publishes
%       % transformations between base_link and camera_depth_frame.
%       % For example, a real or simulated TurtleBot would do that.
%
%        % Connect to the master
%        rosinit("192.168.70.128");
%
%        % Retrieve the transformation tree object
%        % Use struct message format for better performance
%        tftree = ROSTF("DataFormat","struct")
%
%        % Buffer transformations for up to 15 seconds
%        tftree.BufferTime = 15;
%
%        % Wait for the transform that takes data from
%        % "camera_depth_frame" to "base_link". This is blocking until
%        % the transformation is valid.
%        tform = getTransform(tftree,"base_link","camera_depth_frame","Timeout",Inf)
%
%        % Define a point [3 1.5 0.2] in the camera's coordinate frame
%        pt = rosmessage("geometry_msgs/PointStamped","DataFormat","struct");
%        pt.Header.FrameId = 'camera_depth_frame';
%        pt.Point.X = 3;
%        pt.Point.Y = 1.5;
%        pt.Point.Z = 0.2;
%
%        % Transformation is available, so transform the point into the /base_link frame
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
%   See also ros.TransformationTree.

%   Copyright 2014-2021 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            tftree = ros.TransformationTree([], varargin{:});
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        tftree = ros.TransformationTree([], varargin{:});
    end
end
