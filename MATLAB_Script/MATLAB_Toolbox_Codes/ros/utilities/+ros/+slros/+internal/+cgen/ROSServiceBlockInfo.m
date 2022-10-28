classdef ROSServiceBlockInfo < ros.slros.internal.cgen.ROSBlockInfo
%This class is for internal use only. It may be removed in the future.

%ROSServiceBlockInfo Information about a block handling ROS services
%   This is a utility class that encapsulates information about
%   a single ROS block that handles ROS services in a Simulink model.
%
%   See also ROSModelInfo

%   Copyright 2018 The MathWorks, Inc.

    properties
        %SrvType - Service Type
        SrvType

        %CppRosType - Service Type in C++ ROS
        CppRosType

        %SlInputBusName - Bus name of service request input message
        SlInputBusName

        %SlOutputBusName - Bus name of service response output message
        SlOutputBusName
    end

end
