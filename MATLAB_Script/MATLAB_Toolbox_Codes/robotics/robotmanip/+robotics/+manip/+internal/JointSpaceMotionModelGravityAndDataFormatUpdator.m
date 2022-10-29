classdef JointSpaceMotionModelGravityAndDataFormatUpdator < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %JointSpaceMotionModelGravityAndDataFormatUpdator joint space motion
    %   model gravity and data format updator. This is a helper class for
    %   loadrobot function to provide internal access to RigidBodyTreeInternal
    %   property for efficient updation of its properties. The access can
    %   only be provided by a class inheriting from robotmanip access class
    %   robotics.manip.internal.InternalAccess

    %   Copyright 2020 The MathWorks, Inc.    
    
    methods(Static)
        function update(obj,gravity,dataFormat)
            %update updates gravity and data format quickly without
            %   creating any copies.
            
            obj.RigidBodyTreeInternal.Gravity = gravity;
            obj.RigidBodyTreeFormat = dataFormat;
        end
    end
end