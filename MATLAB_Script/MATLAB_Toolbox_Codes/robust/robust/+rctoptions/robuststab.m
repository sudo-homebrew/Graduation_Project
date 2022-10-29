classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) robuststab < ltioptions.Generic
   % Options set for ROBUSTSTAB.
 
   % Author(s): MUSYN
   % Copyright 2010-2011 The MathWorks, Inc.
   properties
      % Displays progress of computations [{'off'} | 'on'].
      Display = 'off'
      % Computes margin sensitivity to individual uncertainties [{'on'} | 'off'].
      Sensitivity = 'on'
      % Percentage variation of uncertainty for sensitivity calculations.
      VaryUncertainty = 25
      % Option string used by MUSSV.
      Mussv = ''
   end
   
   properties (Hidden)
      % Pade replacement order for internal delays.
      PadeN = 4;
   end
   
   
   methods
      
      function op = set.Display(op,V)
         % SET method for Display option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','Display');
         else
            op.Display = V;
         end
      end

      function op = set.Sensitivity(op,V)
         % SET method for Sensitivity option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','Sensitivity');
         else
            op.Sensitivity = V;
         end
      end
         
      function op = set.VaryUncertainty(op,V)
         % SET method for VaryUncertainty option
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.VaryUncertainty = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:VaryUncertainty');
         end
      end
      
      function op = set.Mussv(op,V)
         % SET method for Mussv option
         if isempty(V)
            op.Mussv = '';
         elseif isa(V,'char') && isrow(V)
            op.Mussv = V;
         else
            ctrlMsgUtils.error('Robust:analysis:Mussv');
         end
      end
      
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'robuststab';
      end
   end
   
end
