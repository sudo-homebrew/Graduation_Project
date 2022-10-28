classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) robAnalysis < ltioptions.Generic
   % Options set for ROBSTAB, ROBGAIN, ROBMARGIN.
 
   % Author(s): MUSYN
   % Copyright 2010-2011 The MathWorks, Inc.
   properties
      % Display progress of computation [{'off'} | 'on'].
      Display = 'off'
      % Compute robustness margin over dense frequency grid [{'off'} | 'on'].
      VaryFrequency = 'off'
      % Compute margin sensitivity to individual uncertain elements [{'off'} | 'on'].
      Sensitivity = 'off'
      % Percentage variation of uncertainty for sensitivity calculations.
      SensitivityPercent = 25
      % Option string for MUSSV.
      MussvOptions = ''
   end
   
   properties (Hidden)
      % Pade replacement order for internal delays.
      PadeN = 4;
      % Frequency range of interest
      Focus = [];
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

      function op = set.VaryFrequency(op,V)
         % SET method for VaryFrequency option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','VaryFrequency'))
         end
         op.VaryFrequency = V;
      end
      
      function op = set.Sensitivity(op,V)
         % SET method for Sensitivity option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','Sensitivity'))
         end
         op.Sensitivity = V;
      end
         
      function op = set.SensitivityPercent(op,V)
         % SET method for VaryUncertainty option
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0 && V<Inf
            op.SensitivityPercent = double(V);
         else
            error(message('Robust:analysis:SensitivityPercent'))
         end
      end
      
      function op = set.MussvOptions(op,V)
         % SET method for Mussv option
         if isempty(V)
            op.Mussv = '';
         elseif isa(V,'char') && isrow(V)
            op.Mussv = V;
         else
            error(message('Robust:analysis:MussvOptions'))
         end
      end
      
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'robstab';
      end
   end
   
end
