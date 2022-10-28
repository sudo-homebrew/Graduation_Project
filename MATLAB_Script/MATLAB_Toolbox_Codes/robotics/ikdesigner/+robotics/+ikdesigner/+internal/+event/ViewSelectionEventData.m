classdef (ConstructOnLoad) ViewSelectionEventData < event.EventData
%

%   Copyright 2021 The MathWorks, Inc.

   properties
      SceneObjectKey

      AssocRigidBodyKey
   end
   
   methods
       function data = ViewSelectionEventData(sceneObjKey, assocBodyKey)
           data.SceneObjectKey = sceneObjKey;
           data.AssocRigidBodyKey = assocBodyKey;
       end
   end
end
