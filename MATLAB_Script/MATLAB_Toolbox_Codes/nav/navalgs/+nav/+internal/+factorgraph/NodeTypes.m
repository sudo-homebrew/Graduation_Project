classdef (Hidden) NodeTypes
%   This class is for internal use only. It may be removed in the future.

%NODETYPES Defines the supported node types for factorGraph factors

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (Constant)
        SE3 = "POSE_SE3";
        SE2 = "POSE_SE2";
        Velocity3 = "VEL3";
        Point2 = "POINT_XY";
        Point3 = "POINT_XYZ";
        IMUBias = "IMU_BIAS";
    end
end