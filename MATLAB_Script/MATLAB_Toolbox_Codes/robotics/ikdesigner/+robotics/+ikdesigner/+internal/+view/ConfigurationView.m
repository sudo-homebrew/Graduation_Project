classdef ConfigurationView < handle
    %This class is for internal use only. It may be removed in the future.

    %   ConfigurationView View that displays the displays stored configuration in a user-facing format

    %   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = ?matlab.unittest.TestCase)

        %IsAngularValue Logical vector indicating whether joint entry is an angle
        IsAngularValue
    end

    methods
        function obj = ConfigurationView(isAngularValue)
            %ConfigurationView

            obj.IsAngularValue = isAngularValue;
        end

        function configString = getUserFacingConfigString(obj, rawConfig)

            % Convert revolute joints to degrees
            configVector = rawConfig;
            configVector(obj.IsAngularValue) = rad2deg(rawConfig(obj.IsAngularValue));
            configString = "[" + sprintf("%1.2f ", configVector) + "]";
        end

        function configVector = getInternalConfig(obj, userFacingConfigString)

            % Convert revolute joints to degrees
            configVector = str2num(userFacingConfigString); %#ok<ST2NM>
            configVector(obj.IsAngularValue) = deg2rad(configVector(obj.IsAngularValue));

        end
    end
end
