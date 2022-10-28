classdef (Abstract) ReadXBase < ros.slros.internal.block.ReadXBase
%This class is for internal use only. It may be removed in the future.

%ReadXBase Abstract base class for ROS bus -> array signal blocks

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    methods (Access = protected)
        function setupImpl(obj)
            if coder.target('MATLAB')
                % Executing in MATLAB interpreted mode
                % No setup needed
            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'
                coder.internal.error('ros:slros:sysobj:RapidAccelNotSupported', ['ROS 2 ', obj.IconName, ' Block']);
            elseif coder.target('Rtw')
                % 'Rtw' is executed during model build
            elseif  coder.target('Sfun')
                % 'Sfun'  - SimThruCodeGen target
                % Do nothing. MATLAB System block first does a pre-codegen
                % compile with 'Sfun' target, & then does the "proper"
                % codegen compile with Rtw or RtwForRapid, as appropriate.
            else
                % 'RtwForSim' - ModelReference SIM target
                % 'MEX', 'HDL', 'Custom' - Not applicable to MATLAB System block
                coder.internal.error('ros:slros:sysobj:UnsupportedCodegenMode', coder.target);
            end
        end

        %% validation of the input to Step functions

        function validateInputsImpl(obj,varargin)
        %validateInputsImpl validate the inputs step*h == data length etc
        %   Only valid message types are allowed.
            if coder.target('MATLAB')
                inType = obj.propagatedInputDataType(1);
                validTypes = cellfun(@(type) ros.slros2.internal.bus.Util.rosMsgTypeToBusName(type), ...
                                     obj.ValidMessageTypes, ...
                                     'UniformOutput', false);

                % Display an error if input message type is not supported
                if ~ismember(inType, validTypes)
                    error(message(obj.InvalidTypeID, strjoin(obj.ValidMessageTypes, ', ')));
                end
            end
        end
    end
end
