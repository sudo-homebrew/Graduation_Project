function id = getDefaultDomainID
%This function is for internal use only. It may be removed in the future.

%getDefaultDomainID Return the Domain ID to be used if not specified
%   Uses the value of the environment variable 'ROS_DOMAIN_ID' if valid, or
%   returns the ROS 2 default value of 0 otherwise.

%   Copyright 2019 The MathWorks, Inc.

    environmentVariableName = 'ROS_DOMAIN_ID';
    defaultID = 0;

    % Ignore invalid domain ID values (no error)
    try
        environmentID = str2double(getenv(environmentVariableName));
        validateattributes(environmentID, ...
                           {'numeric'}, ...
                           {'scalar', 'integer', 'nonnegative', '<=', 232})
        id = environmentID;
    catch
        id = defaultID;
    end

end
