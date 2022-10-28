classdef GazeboPublishBlk < matlab.System
    %This function is for internal use only. It may be removed in the future.

    %GazeboPublishBlock send custom message to Gazebo

    %   Copyright 2020 The MathWorks, Inc.

    properties (Nontunable)
        %IPAddress point to the Gazebo plugin server IP or hostname
        IPAddress = 0

        %PortNumber point to the Gazebo plugin server port
        PortNumber = 0

        %Timeout wait time for each call in seconds
        Timeout = 100

        %SampleTime discrete sample time for the block
        SampleTime = 0

        %TopicName name for the particular topic to read from
        TopicName = 0;

        %TopicType type for the particular topic
        TopicType = 0;

        %OutputBusName name for the output bus structure
        OutputBusName = 0;

    end

    properties (Access = private)
        %GazeboClient communication client
        GazeboClient
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
                if ~obj.GazeboClient.initCustomPublisher(obj.TopicName,obj.TopicType)
                    error(message('robotics:robotslgazebo:publishblock:PublishFailed', obj.TopicName));
                end

                if(exist('importGazeboCustomLoadLibrary.m','file'))
                    importGazeboCustomLoadLibrary;
                else
                    error(message('robotics:robotslgazebo:publishblock:CustomDataMissing'));
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

        function outputImpl(obj, cmd)
        % Calculate output

            if coder.target('MATLAB')
                % converts 'uint64' and 'int64' to 'double'
                %cmd = robotics.slgazebo.internal.util.convertToDouble(cmd);

                % convert simulink message into struct of packet string
                cmdString = robotics.slgazebo.internal.util.convertSimulinkMsgToPacketString(cmd, obj.TopicType);

                if( isstruct(cmdString))

                    success = obj.GazeboClient.setCustomMessage(cmdString,obj.TopicName,obj.TopicType);

                    if ~success
                        error(message('robotics:robotslgazebo:publishblock:PublishFailed', obj.TopicName));
                    end

                end
            end
        end

        function flag = isInputDirectFeedthroughImpl(~)
        % Return true if input u is needed to calculate the output at
        % the same time
            flag = true;
        end

        function varargout = getOutputSizeImpl(~)
        % At least one propagator is required so isInputDirectFeedthroughImpl may take
        % precedence over inference result from outputImpl
            varargout = {};
        end
        
        function num = getNumInputsImpl(~)
        % Define total number of inputs for system with optional inputs
            num = 1;
        end

        function num = getNumOutputsImpl(~)
        % Define total number of outputs for system with optional
        % outputs
            num = 0;
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
            coder.internal.errorIf(true, 'robotics:robotslgazebo:blockmask:ApplyCommandSimStateNotSupported');
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
