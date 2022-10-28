classdef GazeboApplyCommandBlk < matlab.System
%This function is for internal use only. It may be removed in the future.

%GazeboApplyCommandBlk applies command to Gazebo

%   Copyright 2019-2021 The MathWorks, Inc.
    properties (Constant)
        CommandApplyLinkWrench = 0;
        CommandApplyJointTorque = 1;
        CommandSetLinkWorldPose = 2;
        CommandSetLinkLinearVelocity = 3;
        CommandSetLinkAngularVelocity = 4;
        CommandSetJointPosition = 5;
        CommandSetJointVelocity = 6;
        NumOfCommands = 7; % Need number of fixed commands for custom message support
    end

    properties (Nontunable)
        %CommandType type for the Gazebo command
        %    0 -> Apply link wrench
        %    1 -> Apply joint torque
        %    2 -> Set Link World Pose
        %    3 -> Set Link Linear Velocity
        %    4 -> Set Link Angular Velocity
        %    5 -> Set Joint Position
        %    6 -> Set Joint Velocity
        CommandType = 0;

        %IPAddress point to the Gazebo plugin server IP or hostname
        IPAddress = 0

        %PortNumber point to the Gazebo plugin server port
        PortNumber = 0

        %Timeout wait time for each call in seconds
        Timeout = 100

        %SampleTime discrete sample time for the block
        SampleTime = 0
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

        function validateInputsImpl(obj, cmd)
        % validate input size and type
            if coder.target('MATLAB')
                % retrieve model name from the input bus
                modelName = char(reshape(cmd.model_name, 1, []));
                validateattributes(modelName, {'char','string'}, {'scalartext'},...
                                   'GazeboApplyCommandBlock', 'modelName');

                if ~isempty(modelName)
                    switch obj.CommandType
                      case obj.CommandApplyLinkWrench
                        linkName = char(reshape(cmd.link_name, 1, []));
                        forceType = char(reshape(cmd.force_type, 1, []));
                        torqueType = char(reshape(cmd.torque_type, 1, []));
                        validateattributes(linkName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'linkName');
                        validateattributes(forceType, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'forceType');
                        validateattributes(torqueType, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'torqueType');
                        validateattributes(cmd.fx, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'fx');
                        validateattributes(cmd.fy, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'fy');
                        validateattributes(cmd.fz, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'fz');
                        validateattributes(cmd.tx, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'tx');
                        validateattributes(cmd.ty, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'ty');
                        validateattributes(cmd.tz, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'tz');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandApplyJointTorque
                        jointName = char(reshape(cmd.joint_name, 1, []));
                        validateattributes(jointName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'jointName');
                        validateattributes(cmd.index, {'uint32'}, {'scalar'},'GazeboApplyCommandBlock', 'index');
                        validateattributes(cmd.effort, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'effort');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandSetLinkWorldPose
                        linkName = char(reshape(cmd.link_name, 1, []));
                        validateattributes(linkName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'linkName');
                        validateattributes(cmd.world_pose.position.x, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.position.x');
                        validateattributes(cmd.world_pose.position.y, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.position.y');
                        validateattributes(cmd.world_pose.position.z, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.position.z');
                        validateattributes(cmd.world_pose.orientation.w, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.orientation.w');
                        validateattributes(cmd.world_pose.orientation.x, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.orientation.x');
                        validateattributes(cmd.world_pose.orientation.y, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.orientation.y');
                        validateattributes(cmd.world_pose.orientation.z, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'world_pose.orientation.z');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandSetLinkLinearVelocity
                        linkName = char(reshape(cmd.link_name, 1, []));
                        validateattributes(linkName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'jointName');
                        validateattributes(cmd.velocity.x, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.x');
                        validateattributes(cmd.velocity.y, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.y');
                        validateattributes(cmd.velocity.z, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.z');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandSetLinkAngularVelocity
                        linkName = char(reshape(cmd.link_name, 1, []));
                        validateattributes(linkName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'linkName');
                        validateattributes(cmd.velocity.x, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.x');
                        validateattributes(cmd.velocity.y, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.y');
                        validateattributes(cmd.velocity.z, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity.z');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandSetJointPosition
                        jointName = char(reshape(cmd.joint_name, 1, []));
                        validateattributes(jointName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'jointName');
                        validateattributes(cmd.index, {'uint32'}, {'scalar'},'GazeboApplyCommandBlock', 'index');
                        validateattributes(cmd.position, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'position');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                      case obj.CommandSetJointVelocity
                        jointName = char(reshape(cmd.joint_name, 1, []));
                        validateattributes(jointName, {'char','string'}, {'scalartext'},'GazeboApplyCommandBlock', 'jointName');
                        validateattributes(cmd.index, {'uint32'}, {'scalar'},'GazeboApplyCommandBlock', 'index');
                        validateattributes(cmd.velocity, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'velocity');
                        validateattributes(cmd.duration.seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.seconds');
                        validateattributes(cmd.duration.nano_seconds, {'double'}, {'scalar'},'GazeboApplyCommandBlock', 'duration.nano_seconds');

                    end
                end
            end
        end


        function outputImpl(obj, cmd)
        % Calculate output

            if coder.target('MATLAB')
                % retrieve model name from the input bus
                modelName = char(reshape(cmd.model_name, 1, []));

                % when model name is empty, incoming data is not yet
                % available, skipping apply phase
                if ~isempty(modelName)
                    switch obj.CommandType
                      case obj.CommandApplyLinkWrench
                        linkName = char(reshape(cmd.link_name, 1, []));
                        forceType = char(reshape(cmd.force_type, 1, []));
                        torqueType = char(reshape(cmd.torque_type, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([cmd.fx,cmd.fy,cmd.fz,...
                                            cmd.tx,cmd.ty,cmd.tz,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["fx","fy","fz",...
                                             "tx","ty","tz",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.applyLinkWrench(...
                            modelName, ...
                            linkName, ...
                            forceType, ...
                            [cmd.fx, cmd.fy, cmd.fz], ...
                            torqueType, ...
                            [cmd.tx, cmd.ty, cmd.tz], ...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToApplyLinkWrench', modelName, linkName));
                        end

                      case obj.CommandApplyJointTorque
                        jointName = char(reshape(cmd.joint_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([double(cmd.index),cmd.effort,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["index","effort",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.applyJointTorque(...
                            modelName, ...
                            jointName, ...
                            cmd.index, ...
                            cmd.effort, ...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToApplyJointTorque', modelName, jointName));
                        end

                      case obj.CommandSetLinkWorldPose
                        linkName = char(reshape(cmd.link_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([cmd.world_pose.position.x,cmd.world_pose.position.y,...
                                            cmd.world_pose.position.z,cmd.world_pose.orientation.w,...
                                            cmd.world_pose.orientation.x,cmd.world_pose.orientation.y,...
                                            cmd.world_pose.orientation.z,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["world_pose.position.x","world_pose.position.y",...
                                             "world_pose.position.z","world_pose.orientation.w",...
                                             "world_pose.orientation.x","world_pose.orientation.y",...
                                             "world_pose.orientation.z",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.setLinkWorldPose(...
                            modelName, ...
                            linkName, ...
                            [cmd.world_pose.position.x, cmd.world_pose.position.y, cmd.world_pose.position.z,...
                             cmd.world_pose.orientation.x,cmd.world_pose.orientation.y,...
                             cmd.world_pose.orientation.z,cmd.world_pose.orientation.w],...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToSetLinkWorldPose', modelName, linkName));
                        end

                      case obj.CommandSetLinkLinearVelocity
                        linkName = char(reshape(cmd.link_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["velocity.x","velocity.y","velocity.z",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.setLinkLinearVelocity(...
                            modelName, ...
                            linkName, ...
                            [cmd.velocity.x, cmd.velocity.y, cmd.velocity.z],...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToSetLinkLinearVelocity', modelName, linkName));
                        end

                      case obj.CommandSetLinkAngularVelocity
                        linkName = char(reshape(cmd.link_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["velocity.x","velocity.y","velocity.z",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.setLinkAngularVelocity(...
                            modelName, ...
                            linkName, ...
                            [cmd.velocity.x, cmd.velocity.y, cmd.velocity.z],...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToSetLinkAngularVelocity', modelName, linkName));
                        end

                      case obj.CommandSetJointPosition
                        jointName = char(reshape(cmd.joint_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([double(cmd.index),cmd.position,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["index","position",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.setJointPosition(...
                            modelName, ...
                            jointName, ...
                            cmd.index, ...
                            cmd.position, ...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToSetJointPosition', modelName, jointName));
                        end

                      case obj.CommandSetJointVelocity
                        jointName = char(reshape(cmd.joint_name, 1, []));

                        % validate cmd inputs and throw error for inf and
                        % nan inputs
                        finteIdx = isfinite([double(cmd.index),cmd.velocity,...
                                            cmd.duration.seconds,cmd.duration.nano_seconds]);
                        if( ~all(finteIdx))
                            cmdFieldNames = ["index","velocity",...
                                             "duration.seconds","duration.nano_seconds"];
                            invalidFieldNames = char(join(cmdFieldNames(~finteIdx)," "));
                            error(message('robotics:robotslgazebo:applycommandblock:ExpectFiniteInput', invalidFieldNames));
                        end

                        success = obj.GazeboClient.setJointVelocity(...
                            modelName, ...
                            jointName, ...
                            cmd.index, ...
                            cmd.velocity, ...
                            uint64(cmd.duration.seconds), ...
                            uint64(cmd.duration.nano_seconds));

                        if ~success
                            error(message('robotics:robotslgazebo:applycommandblock:FailedToSetJointVelocity', modelName, jointName));
                        end
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
