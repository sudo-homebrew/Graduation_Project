classdef (ConstructOnLoad) SolverConstraintEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%SolverConstraintEventData Event data for drafting constraints
%   This event is used to transfer information about the constraints. It
%   can be used to parse existing constraint data into a transferable
%   message to be sent to views that allow constraint editing, and also to
%   transfer that data from the views back to the model. This event is
%   effectively an interface between the view and the constraint object,
%   such that the constraint object itself is only ever created / modified
%   / stored in the associated model.

%   Copyright 2021 The MathWorks, Inc.

   properties
      %Name Name of the constraint
      Name

      %Type Enumeration that indicates the type of constraint
      Type

      %Data Structure that contains the constraint data
      Data

      %Key Key that can be used to query this constraint in the constraints map when it is entered
      Key
   end
   
   methods
       function evtData = SolverConstraintEventData(key, name, type, data)

           evtData.Key = key;
           evtData.Name = name;
           evtData.Type = type;
           evtData.Data = data;
       end
   end
end
