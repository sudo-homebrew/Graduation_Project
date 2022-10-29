classdef credentials < handle
%This class is for internal use only. It may be removed in the future.

%DEVADDRESS Hostname container.
%
% obj = devaddress(hostname, username, password)


% Copyright 2013-2018 The MathWorks, Inc.

    properties (Hidden, SetAccess = private, GetAccess = public)
        Username
        Password
    end

    methods
        function obj = credentials(username, password)
        % Constructor
            narginchk(1, 2);
            obj.Username = username;
            obj.Password = password;
        end
    end

    methods
        function set.Username(obj, value)
            validateattributes(value, {'char'}, {'nonempty', 'row'}, '', 'username');
            obj.Username = strtrim(value);
        end

        function set.Password(obj, value)
            validateattributes(value, {'char'}, {}, '', 'password');
            obj.Password = value;
        end
    end
end % classdef

%[EOF]
