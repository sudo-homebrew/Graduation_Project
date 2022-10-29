classdef GazeboReadBlk < matlab.System
    %This function is for internal use only. It may be removed in the future.

    %GazeboRead reads from Gazebo and output its content

    %   Copyright 2019-2020 The MathWorks, Inc.

    properties (Nontunable)
        %OutputBusName name for the output bus structure
        OutputBusName = 0;

        %SampleTime discrete sample time for the block
        SampleTime = 0.01;

        %TopicName name for the particular topic to read from
        TopicName = 0;

        %TopicType type for the particular topic
        TopicType = 0;

        %IPAddress point to the Gazebo plugin server IP or hostname
        IPAddress = 0

        %PortNumber point to the Gazebo plugin server port
        PortNumber = 0

        %Timeout wait time for each call in seconds
        Timeout = 100
    end

    properties (Access = private)
        %GazeboClient communication client
        GazeboClient

        %Reader message reader to receive topic message from Gazebo
        Reader
    end


    methods(Access = protected)
        function setupImpl(obj)
        % Perform one-time calculations, such as computing constants
            obj.reset();
        end

        function resetImpl(obj)
        % Initialize / reset discrete-state properties

            if coder.target('MATLAB')
                validateattributes(obj.IPAddress, {'string', 'char'}, {'scalartext'}, 'GazeboStepping', 'HostIP');
                validateattributes(obj.PortNumber, {'numeric'}, {'integer', 'positive', '<=', 65536}, 'GazeboStepping', 'HostPort');
                validateattributes(obj.Timeout, {'numeric'}, {'scalar', 'positive'}, 'GazeboStepping', 'Timeout');
                obj.GazeboClient = robotics.internal.GazeboClient;
                obj.GazeboClient.connect(convertStringsToChars(obj.IPAddress), obj.PortNumber, obj.Timeout*1000);
                obj.Reader = robotics.gazebo.internal.GazeboReader(obj.GazeboClient);
                if ~obj.Reader.subscribeIfNeeded(obj.TopicType, obj.TopicName)
                    error(message('robotics:robotslgazebo:readblock:SubscribeFailed', obj.TopicName));
                end
            end
        end

        function releaseImpl(obj)
        % Release resources, such as file handles

            if coder.target('MATLAB')
                if ~isempty(obj.GazeboClient)
                    obj.GazeboClient.shutdown();
                end
            end
        end

        function [isNew, result] = outputImpl(obj)
        % Calculate output

            if coder.target('MATLAB')
                % Read message from Gazebo based on topic type and name
                if ~isempty(obj.getCurrentTime())
                    simTime = obj.getCurrentTime();
                    seconds = floor(simTime);
                    remainder = simTime-seconds;
                    obj.GazeboClient.setSimulationTime(uint64(seconds), uint64(remainder*1e9));
                end
                [isNew, result] = obj.Reader.read(obj.TopicType, obj.TopicName);

                % Convert string fields into uint8 array because Simulink
                % MATLAB System block doesn't support string input/output
                result = robotics.slgazebo.internal.util.convertStringToUint8(result);

                % pad empty arrays in result for var-sized signals with a
                % single zero because Simulink doesn't allow empty array in
                % var-sized signals
                result = robotics.slgazebo.internal.util.padSignals(result);
            end
        end

        function num = getNumInputsImpl(~)
        % Define total number of inputs for system with optional inputs
            num = 0;
        end

        function num = getNumOutputsImpl(~)
        % Define total number of outputs for system with optional
        % outputs
            num = 2;
        end

        function [outIsNew, outBus] = getOutputSizeImpl(~)
        % Return size for each output port
            outIsNew = [1 1];
            outBus = [1 1];
        end

        function [outIsNew, outBus] = getOutputDataTypeImpl(obj)
        % Return data type for each output port
            outIsNew = 'logical';
            outBus = obj.OutputBusName;
        end

        function [outIsNew, outBus] = isOutputComplexImpl(~)
        % Return true for each output port with complex data
            outIsNew = false;
            outBus = false;
        end

        function [outIsNew, outBus] = isOutputFixedSizeImpl(~)
        % Return true for each output port with fixed size
            outIsNew = true;
            outBus = true;
        end

        function sts = getSampleTimeImpl(obj)
        %getSampleTimeImpl supports discrete sample time only
            sts = obj.createSampleTime("Type", "Discrete", ...
                                       "SampleTime", obj.SampleTime, 'OffsetTime', 0);
        end

        function s = saveObjectImpl(obj)
        % We don't save SimState, since there is no way save & restore the
        % co-simulation client object. The errorIf() below will ensure that
        % FastRestart and back-stepping cannot be used in a model with ROS
        % blocks
            coder.internal.errorIf(true, 'robotics:robotslgazebo:blockmask:ReadSimStateNotSupported');
            s = saveObjectImpl@matlab.System(obj);
        end

        function loadObjectImpl(obj,s,wasLocked)
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
        % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
