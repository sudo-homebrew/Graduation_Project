classdef (ConstructOnLoad) PoseEventData < event.EventData
%

%   Copyright 2021 The MathWorks, Inc.

   properties
      Pose
   end
   
   methods
      function data = PoseEventData(pose)
         data.Pose = pose;
      end
   end
end
