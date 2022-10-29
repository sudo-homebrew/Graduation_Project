classdef GazeboPacer < matlab.System
    %This class is for internal use only. It may be removed in the future.

    %GazeboPacer pace Gazebo simulator

    %   Copyright 2019-2020 The MathWorks, Inc.

    properties (Nontunable)
        %IPAddress point to the Gazebo plugin server IP or hostname
        IPAddress = 0

        %PortNumber point to the Gazebo plugin server port
        PortNumber = 0

        %Timeout wait time for each call in seconds
        Timeout = 100

        %SampleTime for this block, must be discrete fixed value
        SampleTime = 0.001

        %ResetBehavior configure how to reset Gazebo simulation
        ResetBehavior = 0

        %CoSimClientID unique ID used for co-simulation. Set to bdroot in
        %Simulink
        CoSimClientID = 0
    end

    properties (Access = private)
        %GazeboClient built-in object to interact with Gazebo server
        GazeboClient

        %StepStatus indicates whether each step call to Gazebo is
        %successful
        StepStatus
        
        %GazeboStepSize contains the max step size of the physics solver
        %used by Gazebo
        GazeboStepSize
        
        %NumSteps - number of steps to take in Gazebo for each Simulink step
        NumSteps
    end

    methods(Access = protected)
        function setupImpl(obj)
        % setupImpl call reset to initialize Gazebo client
            obj.reset();
        end

        function resetImpl(obj)
        % initializes/Reset Gazebo client and connect

            if coder.target('MATLAB')
                validateattributes(obj.IPAddress, {'string', 'char'}, {'scalartext'}, 'GazeboStepping', 'HostIP');
                validateattributes(obj.PortNumber, {'numeric'}, {'integer', 'positive', '<=', 65536}, 'GazeboStepping', 'HostPort');
                validateattributes(obj.Timeout, {'numeric'}, {'scalar', 'positive'}, 'GazeboStepping', 'Timeout');
                obj.GazeboClient = robotics.internal.GazeboClient;
                obj.GazeboClient.connect(convertStringsToChars(obj.IPAddress), obj.PortNumber, obj.Timeout*1000);
                obj.GazeboClient.requestCoSim(obj.CoSimClientID);

                obj.GazeboStepSize = obj.GazeboClient.getMaxStepSize();
                if isnan(obj.GazeboStepSize)
                    error(message('robotics:robotslgazebo:stepblock:GazeboMaxStepSizeInvalid'));
                end
                
                obj.NumSteps = obj.SampleTime/obj.GazeboStepSize;
                % check whether numSteps is an integer number. It is computed
                % by sampleTime/GazeboMaxStepSize
                if floor(obj.NumSteps) ~= obj.NumSteps
                    % If called from Simulink block, get the path to the mask
                    % block that contains this System block and fill it in the
                    % error message
                    blockPath = gcb;
                    if ~isempty(blockPath)
                        blockPath = get_param(gcb, 'Parent');
                    end
                    error(message('robotics:robotslgazebo:stepblock:SampleTimeNotMultiple', blockPath, num2str(obj.GazeboStepSize), num2str(obj.SampleTime)));
                end

                switch obj.ResetBehavior
                  case 0
                    resetResult = obj.GazeboClient.resetTime();
                  case 1
                    resetResult = obj.GazeboClient.resetAll();
                end

                if ~resetResult
                    error(message('robotics:robotslgazebo:stepblock:ResetFailed').getString());
                end
            end

            obj.StepStatus = uint8(0);
        end

        function releaseImpl(obj)
        % Close client connection on release
            if coder.target('MATLAB')
                if ~isempty(obj.GazeboClient)
                    obj.GazeboClient.shutdown();
                end
            end
        end

        function validatePropertiesImpl(obj)
        % Validate related or interdependent property values

            validateattributes(obj.SampleTime, {'numeric'}, {'scalar', 'positive'}, 'GazeboStepping', 'SampleTime');
            validateattributes(obj.ResetBehavior, {'numeric'}, {'scalar', 'integer', '<=', 1, '>=', 0}, 'GazeboStepping', 'ResetBehavior');
        end

        function num = getNumInputsImpl(~)
        % Define total number of inputs for system with optional inputs
            num = 0;
        end

        function num = getNumOutputsImpl(~)
        % Define total number of outputs for system with optional
        % outputs
            num = 1;
        end

        function name = getOutputNamesImpl(~)
        % Return output port names for System block
            name = 'StepStatus';
        end

        function out = getOutputSizeImpl(~)
        % Step status is always fixed size scalar
            out = [1 1];
        end

        function out = getOutputDataTypeImpl(~)
        % Return step status as logical
            out = "uint8";
        end

        function out = isOutputComplexImpl(~)
        % Output is always non-complex
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
        % Step status is always fixed size scalar
            out = true;
        end

        function sts = getSampleTimeImpl(obj)
        % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.SampleTime, 'OffsetTime', 0);
        end

        function updateImpl(obj)
        % Step the Gazebo simulator during update phase
            if coder.target('MATLAB')
                if obj.GazeboClient.step(double(obj.NumSteps))
                    obj.StepStatus = uint8(0);
                else
                    obj.StepStatus = uint8(1);
                end
            end
        end

        function y = outputImpl(obj)
        % Output the step status
            y = obj.StepStatus;
        end
        
        function s = saveObjectImpl(obj)
        % We don't save SimState, since there is no way save & restore the
        % co-simulation client object. The errorIf() below will ensure that
        % FastRestart and back-stepping cannot be used in a model with ROS
        % blocks
            coder.internal.errorIf(true, 'robotics:robotslgazebo:blockmask:PacerSimStateNotSupported');
            s = saveObjectImpl@matlab.System(obj);
        end

        function loadObjectImpl(obj,s,wasLocked)
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
        % Only allow interpreted execution for driving Gazebo simulator
            simMode = "Interpreted execution";
        end
    end
end
