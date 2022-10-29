function varargout = rosShowDetails(msg)
%rosShowDetails Print the details of a ROS or ROS 2 message struct recursively
%   rosShowDetails(MSG) recursively generates a display string for the
%   data contents of the input ROS or ROS 2 message. The complete string
%   is returned in VARARGOUT.
%
%   Example:
%       % Create a ROS message
%       msg = rosmessage("geometry_msgs/PointStamped","DataFormat","struct");
%       msg.Point.X = 1;
%       msg.Point.Y = 2;
%       msg.Point.Z = 3;
%
%       % Show message details
%       rosShowDetails(msg);

%   Copyright 2020 The MathWorks, Inc.

    validateattributes(msg, {'struct'}, {'scalar'}, ...
                       'rosShowDetails', 'msg');

    % Determine if the input message is a ROS or ROS 2 message
    allFields = fieldnames(msg);
    firstField = allFields{1};
    secondField = allFields{2};
    capExpr = '[A-Z]';
    firstCapStartIdx = regexp(firstField,capExpr,'once');
    secondCapStartIdx = regexp(secondField,capExpr,'once');

    if ~isempty(firstCapStartIdx) && ~isempty(secondCapStartIdx)
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    varargout = specialMsgUtil.showDetails(msg);
    varargout = {varargout};
end
