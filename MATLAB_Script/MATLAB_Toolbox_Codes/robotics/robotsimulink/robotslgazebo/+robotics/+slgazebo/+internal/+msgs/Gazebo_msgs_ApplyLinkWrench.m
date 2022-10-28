classdef Gazebo_msgs_ApplyLinkWrench < robotics.slgazebo.internal.msgs.Gazebo_Msgs
    %This function is for internal use only. It may be removed in the future.
    
    %Gazebo_msgs_ApplyLinkWrench defines the apply link wrench command
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        
        MessageType = 'gazebo_msgs/ApplyLinkWrench';
        model_name = ''
        link_name = ''
        force_type = ''
        fx = 0
        fy = 0
        fz = 0
        torque_type = ''
        tx = 0
        ty = 0
        tz = 0
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time
        
    end
    
    methods
        
        function obj = Gazebo_msgs_ApplyLinkWrench(~)
            
            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0; 
            durationTime.nano_seconds = 0;
            obj.duration = durationTime;
            
        end
        
    end
    
end