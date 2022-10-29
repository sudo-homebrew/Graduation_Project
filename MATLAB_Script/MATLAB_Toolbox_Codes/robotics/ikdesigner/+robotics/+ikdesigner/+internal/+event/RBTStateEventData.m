classdef (ConstructOnLoad) RBTStateEventData < event.EventData
%

%   Copyright 2021 The MathWorks, Inc.

   properties
      Config
      
      TransformTree
      
      SelectedBodyIdx
      
      EEBodyIdx
   end
   
   methods
      function data = RBTStateEventData(config, ttree)
         data.Config = config;
         
         data.TransformTree = ttree;
      end
   end
end
