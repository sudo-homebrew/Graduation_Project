classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) ...
      hinfsyn < ltioptions.Generic
   % Options set for HINFSYN.
   
   %   Copyright 2017 The MathWorks, Inc.
   properties
      % Display mode
      Display = 'off';
      % Method ('RIC' or 'LMI')
      Method = 'RIC'; 
      % Relative tolerance on optimal performance
      RelTol = 1e-2;
      % Absolute tolerance on optimal performance
      AbsTol = 1e-6;
      % Scale plant state and u,y channels for maximum accuracy
      AutoScale = 'on';
      % Automatic regularization
      Regularize = 'on';
      % Limit controller gains
      LimitGain = 'on';
      % Penalization of ||R||+||S||
      LimitRS = 0;
      % Tolertance for triggering reduced-order synthesis
      TolRS = 1e-3;
      % Frequency at which entropy is evaluated
      S0 = Inf;
   end
     
   methods
      
      function op = set.Display(op,V)
         % SET method for Display option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','Display'))
         end
         op.Display = V;
      end
      
      function op = set.Method(op,V)
         % SET method for Display option
         V = ltipack.matchKey(V,{'RIC','LMI','MAXE'}); % ,'RICAP','RICNOREGULARIZE','PEN','PENNOREGULARIZE'});
         if isempty(V)
            error(message('Robust:design:hinfsyn1'))
         end
         op.Method = V;
      end
      
      function op = set.RelTol(op,V)
         % SET method for RelTol option
         if isnumeric(V) && isscalar(V) && isreal(V) && isfinite(V) && V>0
            op.RelTol = double(V);
         else
            error(message('Robust:design:hinfsyn3'))
         end
      end
      
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
      
      function op = set.LimitGain(op,V)
         % SET method for LimitGain option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','LimitGain'))
         end
         op.LimitGain = V;
      end
      
      function op = set.AbsTol(op,V)
         % SET method for AbsTol option
         if isnumeric(V) && isscalar(V) && isreal(V) && isfinite(V) && V>=0
            op.AbsTol = double(V);
         else
            error(message('Robust:design:hinfsyn2'))
         end
      end
      
      function op = set.LimitRS(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0 && V<=1
            op.LimitRS = double(V);
         else
            error(message('Robust:design:hinfsyn4'))
         end
      end
      
      function op = set.TolRS(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0 && V<Inf
            op.TolRS = double(V);
         else
            error(message('Robust:design:hinfsyn11'))
         end
      end
      
      function op = set.S0(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.S0 = double(V);
         else
            error(message('Robust:design:hinfsyn19'))
         end
      end
      
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'hinfsyn';
      end
   end
   
end
