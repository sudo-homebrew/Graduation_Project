classdef Gazebo_msgs_Time < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_Time is the template for representing time

%   Copyright 2019 The MathWorks, Inc.
    
    properties
        
        MessageType = 'gazebo_msgs/Time'    
        seconds 
        nano_seconds
    end
    
    methods
        
        function obj = Gazebo_msgs_Time(~)
            
            obj.seconds = 0;
            obj.nano_seconds= 0;
            
        end        
    end       
end