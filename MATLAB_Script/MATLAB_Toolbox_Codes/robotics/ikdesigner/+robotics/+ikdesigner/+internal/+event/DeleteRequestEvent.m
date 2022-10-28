classdef (ConstructOnLoad) DeleteRequestEvent < event.EventData
%This class is for internal use only and may be removed in a future release.

%DeleteRequestEvent Event data for deleting objects in the app

%   Copyright 2021 The MathWorks, Inc.

   properties
      %Key Key of the object to be deleted
      %   The key is used to remove the object from a containing
      %   containers.Map object. In the app context, this may e.g. refer to
      %   a scene object key, or to a constraint key.
      Key
   end
   
   methods
       function data = DeleteRequestEvent(objKey)
           data.Key = objKey;
       end
   end
end
