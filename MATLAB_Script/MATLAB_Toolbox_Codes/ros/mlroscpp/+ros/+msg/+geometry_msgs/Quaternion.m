classdef Quaternion < ros.msggen.geometry_msgs.Quaternion
%QUATERNION Custom MATLAB implementation of geometry_msgs/Quaternion type
%   This class adds functionality for returning a QUATERNION object.
%
%   Quaternion methods:
%      readQuaternion    - Returns the quaternion as a QUATERNION object

%   Copyright 2017-2020 The MathWorks, Inc.

    methods
        function obj = Quaternion(varargin)
        %Quaternion Constructor
        %   The arguments feed straight into the generated Quaternion class.

            obj@ros.msggen.geometry_msgs.Quaternion(varargin{:});
        end

        function q = readQuaternion(obj)
        %READQUATERNION Returns the quaternion as a QUATERNION object
        %   Q = READQUATERNION(OBJ) returns a quaternion, Q,
        %   represented by a QUATERNION object, from the quaternion
        %   message object, OBJ.
        %
        %   See also quaternion.
            q = quaternion(obj.W, obj.X, obj.Y, obj.Z);
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.geometry_msgs.Quaternion.empty(0,1);
                return
            end

            % Create an empty object
            obj = ros.msg.geometry_msgs.Quaternion(strObj);
        end
    end

end
