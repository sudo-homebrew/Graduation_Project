classdef Gazebo_msgs_ApplyJointTorque < robotics.slgazebo.internal.msgs.Gazebo_Msgs
    %This function is for internal use only. It may be removed in the future.
    
    %Gazebo_msgs_ApplyJointTorque defines the apply joint torque command
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        
        MessageType = 'gazebo_msgs/ApplyJointTorque';
        model_name = ''
        joint_name = ''
        index uint32 = 0
        effort = 0
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time
        
    end
    
    methods
        
        function obj = Gazebo_msgs_ApplyJointTorque(~)
            
            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;
            
            obj.duration = durationTime;
            
        end
        
    end
    
end