classdef GazeboBlankMessageBlk < matlab.System
    %This class is for internal use only. It may be removed in the future.

    %GazeboBlankMessageBlk outputs a blank message

    %   Copyright 2019-2020 The MathWorks, Inc.

    properties (Nontunable)
        %OutputBusName name for the output bus structure
        OutputBusName = 0;

        %SampleTime discrete sample time for the block
        SampleTime = 0.01;

        %MessageType
        MessageType
    end

    properties (Access = private)
        %TemplateStruct empty message structure
        TemplateStruct
    end


    methods(Access = protected)
        function setupImpl(obj)
        % Perform one-time calculations, such as computing constants
            obj.reset();
        end

        function resetImpl(obj)
        % Initialize / reset discrete-state properties

            if coder.target('MATLAB')
                switch obj.MessageType
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandApplyLinkWrench
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_ApplyLinkWrench;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandApplyJointTorque
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_ApplyJointTorque;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandSetLinkWorldPose
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_SetLinkWorldPose;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandSetLinkLinearVelocity
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_SetLinkLinearVelocity;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandSetLinkAngularVelocity
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_SetLinkAngularVelocity;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandSetJointPosition
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_SetJointPosition;
                  case robotics.slgazebo.internal.GazeboApplyCommandBlk.CommandSetJointVelocity
                    template = robotics.slgazebo.internal.msgs.Gazebo_msgs_SetJointVelocity;
                  otherwise
                    template = robotics.slgazebo.internal.util.GetGazeboCustomMessageBlkTemplate...
                        (obj.MessageType-robotics.slgazebo.internal.GazeboApplyCommandBlk.NumOfCommands);
                end

                obj.TemplateStruct = robotics.slgazebo.internal.util.convertMsgClassToStruct(template);
            end
        end

        function result = outputImpl(obj)
        % Calculate output

            if coder.target('MATLAB')
                % Read message from Gazebo based on topic type and name
                result = obj.TemplateStruct;
            end
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

        function outBus = getOutputSizeImpl(~)
        % Return size for each output port
            outBus = [1 1];
        end

        function outBus = getOutputDataTypeImpl(obj)
        % Return data type for each output port
            outBus = obj.OutputBusName;
        end

        function outBus = isOutputComplexImpl(~)
        % Return true for each output port with complex data
            outBus = false;
        end

        function outBus = isOutputFixedSizeImpl(~)
        % Return true for each output port with fixed size
            outBus = true;
        end

        function sts = getSampleTimeImpl(obj)
        %getSampleTimeImpl supports discrete sample time only
            sts = obj.createSampleTime("Type", "Discrete", ...
                                       "SampleTime", obj.SampleTime, 'OffsetTime', 0);
        end

    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
        % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
