classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) wcmargin < ltioptions.Generic
   % Options set for WCMARGIN.
   
   % Copyright 2010-2011 The MathWorks, Inc.
   properties
      % Compute sensitivity to individual uncertain elements ['on' | {'off'}].
      Sensitivity = 'off'
      % Absolute tolerance for upper/lower bound calculations.
      AbsTol = 0.02
      % Relative tolerance for upper/lower bound calculations.
      RelTol = 0.05
   end
   
      
   methods
      
      function op = set.Sensitivity(op,V)
         % SET method for Sensitivity option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','Sensitivity')
         else
            op.Sensitivity = V;
         end
      end
      
      function op = set.AbsTol(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0
            op.AbsTol = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:NonnegativeScalar','AbsTol')
         end
      end
      
      function op = set.RelTol(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0
            op.RelTol = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:NonnegativeScalar','RelTol')
         end
      end
            
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'wcmargin';
      end
   end
   
end
