classdef ServiceCallErrorCode < uint8
%This class is for internal use only. It may be removed in the future.

%ServiceCallErrorCodes Call service on ROS network and receive response
%   All the enumerations are prefixed with "SL", since the property
%   names will become defines in the generated C++ code and we want to
%   avoid name collisions.
%
%   See also ServiceCaller.

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

    enumeration
        %SLSuccess - The service response was successfully retrieved.
        SLSuccess (0)

        %SLConnectionTimeout - The connection to the server was not established within timeout
        SLConnectionTimeout (1)

        %SLCallFailure - The call to the service server failed
        SLCallFailure (2)

        %SLOtherError - Some other error occurred
        SLOtherError (3)
    end
end
