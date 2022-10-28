classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) ...
      h2syn < ltioptions.Generic
   % Options set for H2SYN.
   
   %   Copyright 2010-2017 The MathWorks, Inc.
   properties
      % Scale plant state and u,y channels for maximum accuracy
      AutoScale = 'on';
      % Automatic regularization
      Regularize = 'on';
   end
   
   methods
      
      function op = set.AutoScale(op,V)
         % SET method for AutoScale option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','AutoScale'))
         end
         op.AutoScale = V;
      end
            
      function op = set.Regularize(op,V)
         % SET method for Regularize option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','Regularize'))
         end
         op.Regularize = V;
      end
            
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'h2syn';
      end
   end
   
end
