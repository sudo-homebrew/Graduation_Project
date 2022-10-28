classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) robustperf < rctoptions.robuststab
   % Options set for ROBUSTPERF
 
   % Copyright 2010-2011 The MathWorks, Inc.
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'robustperf';
      end
   end
   
end
