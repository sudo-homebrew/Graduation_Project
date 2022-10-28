classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) wcgain < ltioptions.Generic
   % Options set for WCGAIN.
   
%   Copyright 2010-2012 The MathWorks, Inc.
   properties
      % Compute sensitivity to individual uncertain elements [{'on'} | 'off'].
      Sensitivity = 'on'
      % Percentage variation of uncertainty for sensitivity calculations.
      VaryUncertainty = 25
      % Only compute lower bound for worst-case gain [{'off'} | 'on'].
      LowerBoundOnly = 'off'
      % Only compute worst-case peak (over frequency) gain [{'on'} | 'off'].
      MaxOverFrequency = 'on'
      % Only compute worst-case gain across all array elements [{'on'} | 'off'].
      MaxOverArray = 'on'
      % Absolute tolerance for upper/lower bound calculations.
      AbsTol = 0.02
      % Relative tolerance for upper/lower bound calculations.
      RelTol = 0.05
      % Absolute threshold for lower bound calculations.
      AbsMax = 5
      % Relative threshold for lower bound calculations.
      RelMax = 20
      % Number of lower bound searches (positive integer).
      NSearch = 2
   end
   
   properties (Hidden)
      % Pade replacement order for internal delays.
      PadeN = 4;
      FreqRange = [];
      % Number of cycles in lower bound search (positive integer).
      MaxCnt = 3
      % Maximum computation time allowed (seconds).
      MaxTime = 720
      % Maximum number of branches per problem (nonnegative integer).
      MaxBranches = 50
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

      function op = set.VaryUncertainty(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.VaryUncertainty = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:VaryUncertainty')
         end
      end
      
      function op = set.LowerBoundOnly(op,V)
         % SET method for Sensitivity option
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','LowerBoundOnly')
         else
            op.LowerBoundOnly = V;
         end
      end
      
      function op = set.MaxOverFrequency(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','MaxOverFrequency')
         else
            op.MaxOverFrequency = V;
         end
      end
      
      
      function op = set.MaxOverArray(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','MaxOverArray')
         else
            op.MaxOverArray = V;
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
      
      function op = set.AbsMax(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.AbsMax = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:PositiveScalar','AbsMax')
         end
      end
      
      function op = set.RelMax(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.RelMax = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:PositiveScalar','RelMax')
         end
      end
      
      function op = set.NSearch(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && rem(V,1)==0 && V>0
            op.NSearch = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:PositiveInteger','NSearch');
         end
      end
            
%       function op = set.PadeN(op,V)
%          if isnumeric(V) && isscalar(V) && isreal(V) && rem(V,1)==0 && V>=1
%             op.PadeN = double(V);
%          else
%             ctrlMsgUtils.error('Robust:analysis:PadeN');
%          end
%       end
%       
%       function op = set.FreqRange(op,V)
%          if isa(V,'cell') && isequal(size(V),[1 2]) && ...
%                (isnumeric(V{1}) && isreal(V{1}) && V{1}>=0 && ...
%                isnumeric(V{2}) && isreal(V{2}) && V{2}>=V{1})
%                op.FreqRange = V;
%          else
%             ctrlMsgUtils.error('Robust:analysis:FreqRange');
%          end
%       end
            
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'wcgain';
      end
   end
   
end
