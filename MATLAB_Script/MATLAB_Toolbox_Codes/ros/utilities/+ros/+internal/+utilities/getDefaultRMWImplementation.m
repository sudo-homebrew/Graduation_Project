function rmwImpl = getDefaultRMWImplementation
%This function is for internal use only. It may be removed in the future.

%getDefaultRMWImplementation Return the ROS Middleware Implementation to be used if not specified
%   Uses the value of the environment variable 'RMW_IMPLEMENTATION' if valid, or
%   returns the ROS 2 default value of 'rmw_fastrtps_cpp' otherwise.

%   Copyright 2021 The MathWorks, Inc.

    environmentVariableName = 'RMW_IMPLEMENTATION';
    defaultRMW = 'rmw_fastrtps_cpp';

    % Ignore invalid rmw Implementation values (no error)
    try
        environmentRMW = getenv(environmentVariableName);
        if isempty(environmentRMW)
            rmwImpl = defaultRMW;
            return;
        end
        validateattributes(environmentRMW, ...
                           {'char', 'string'}, ...
                           {'scalartext'})
        rmwImpl = environmentRMW;
    catch
        rmwImpl = defaultRMW;
    end

end
