classdef ROSMessageBlockInfo < ros.slros.internal.cgen.ROSBlockInfo
%This class is for internal use only. It may be removed in the future.

%ROSMessageBlockInfo is a utility class that encapsulates information about
%   a single ROS block that handles ROS messages in a Simulink model.
%
%   See also: cgen.ROSModelInfo

%   Copyright 2015-2018 The MathWorks, Inc.

    properties
        MsgType
        CppRosType
        SlBusName
    end

end
