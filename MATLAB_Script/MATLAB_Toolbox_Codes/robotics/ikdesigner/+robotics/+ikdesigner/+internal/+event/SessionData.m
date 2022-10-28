classdef (ConstructOnLoad) SessionData < event.EventData
    %This function is for internal use only. It may be removed in the future.
    
    %SessionData EventData object used to initiate an app session
    
    %   Copyright 2021 The MathWorks, Inc.
    
   properties
       %RigidBodyTree Handle to rigid body tree object
      RigidBodyTree
   end
   
   methods
      function data = SessionData(robot)
          %SessionData Constructor
         data.RigidBodyTree = robot;
      end
   end
end