function [cart, cartAngles] = rosReadCartesian(msg, varargin)
%rosReadCartesian Return Cartesian coordinates from ROS/ROS 2 message struct
%   [CART,CARTANGLES] = rosReadCartesian(MSG) converts the polar
%   measurements of the laser scan into Cartesian coordinates CART. This
%   function uses the meta data in the message, such as the
%   angular resolution and opening angle of the laser scanner.
%   The coordinates are returned in meters.
%
%   The coordinates follow the standard ROS convention of: x
%   axis forward and y axis left.
%
%   CART = rosReadCartesian(___,Name,Value) allows the specification of
%   optional name/value pairs:
%      "RangeLimits"  -  specifies the minimum and maximum values of valid
%                        ranges as a [min max] vector. All ranges smaller
%                        than min or larger than max will be ignored during
%                        the conversion to Cartesian coordiantes.
%                        Default: [MSG.RangeMin MSG.RangeMax]
%
%   [CART,CARTANGLES] = rosReadCartesian(___) also returns the
%   scan angles CARTANGLES that are associated with each of the
%   Cartesian coordinates. The angles are measured counter-clockwise
%   around the positive z axis and will be in radians.
%   CARTANGLES will be returned as an N-by-1 vector and CART
%   will be an N-by-2 matrix.
%
%   The XY coordinates will be returned as an N-by-2 matrix,
%   where N is less than or equal to the number of range
%   readings. All NaN or infinite ranges will be ignored for this
%   conversion. All range values outside of the laser scanner's
%   defined [RangeMin RangeMax] limits will also be ignored.
%
%   Example:
%       % Create a LaserScan message
%       msg = rosmessage("sensor_msgs/LaserScan","DataFormat","struct");
%       msg.AngleMin = -0.5467;
%       msg.AngleMax = 0.5467;
%       msg.AngleIncrement = 0.0017;
%       msg.RangeMin = 0.45;
%       msg.RangeMax = 10;
%       msg.Ranges = single(linspace(0.45,10,640)');
%
%       % Read cartesian coordinate from the message
%       [cart,cartAngles] = rosReadCartesian(msg);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadCartesian');

    % Code generation only supports sensor_msgs/LaserScan as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/LaserScan'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadCartesian','sensor_msgs/LaserScan');

        % Code generation does not support empty message struct
        if isfield(msg,'Ranges')
            coder.internal.assert(~isempty(msg.Ranges),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadCartesian');
        else
            coder.internal.assert(~isequal(msg.ranges,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadCartesian');
        end
    end

    % Get cart and cartAngles from input message
    if isfield(msg, 'Ranges')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    [cart, cartAngles] = specialMsgUtil.readCartesian(msg, varargin{:});
end
