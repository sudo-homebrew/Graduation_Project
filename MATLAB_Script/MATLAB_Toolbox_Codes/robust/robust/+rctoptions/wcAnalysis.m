classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) wcAnalysis < rctoptions.robAnalysis
   % Options set for WCGAIN, WCSIGMAPLOT, WCDISKMARGIN.
 
   % Author(s): MUSYN
   % Copyright 2010-2016 The MathWorks, Inc.
   properties
      % Uncertainty level (default=1)
      ULevel = 1
   end
   
   
   methods
      
      function op = set.ULevel(op,V)
         % SET method for ULevel option
         if ~(isnumeric(V) && isscalar(V) && isreal(V) && V>0 && V<Inf)
            error(message('Robust:analysis:ULevel'))
         end
         op.ULevel = V;
      end
      
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'wcgain';
      end
   end
   
end
