classdef (ConstructOnLoad) ConstraintDisplayEvent < event.EventData
%ConstraintDisplayEvent Constraint display update

%   Copyright 2021 The MathWorks, Inc.

   properties (Abstract, Constant)
      AxesOverlayConstructorHandle
   end

   properties (Abstract)
      UpdateType
   end
   
   methods
       function data = ConstraintDisplayEvent(updateType)
           data.UpdateType = updateType;
       end
   end
end
