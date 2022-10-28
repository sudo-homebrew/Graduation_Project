classdef DataContainer < handle
%This class is for internal use only. It may be removed in the future.

%DataContainer Helper handle class for storing user data
%   Allows for accessing and operating on the same data when passed between
%   functions and callbacks.

%   Copyright 2021 The MathWorks, Inc.

    properties
        %UserData - Storage property for any data type or quantity
        UserData
    end

    properties (Constant, Access = private)
        %DefaultUserData - Default value of UserData property
        DefaultUserData = [];
    end

    methods
        function obj = DataContainer(userData)
        %DataContainer Construct object for storing user data
        %   It is recommended to set the data stored with default values of
        %   the same type for code generation compatibility.

            if nargin < 1
                userData = obj.DefaultUserData;
            end
            obj.UserData = userData;
        end
    end
end
