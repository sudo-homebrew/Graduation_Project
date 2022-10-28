classdef (CaseInsensitiveProperties = true, TruncatedProperties = true)...
      musyn < ltioptions.Generic
   % Options set for MUSYN
   
   %   Copyright 2019 The MathWorks, Inc.
   properties
      % Display level
      Display = 'short';
      % Maximum number of D-K iterations to perform.
      MaxIter = 10;
      % Target performance (stopping condition)
      TargetPerf = 0;
      % Tolerance on performance (stopping condition)
      TolPerf = 0.01;
      % Real/complex mu-synthesis [{'off'} | 'on']
      MixedMU = 'off';
      % Use full D,G
      FullDG = true(1,2);
      % State order for fitting D/G scaling data.
      FitOrder = [5 2];
      % Frequency grid for mu analysis.
      FrequencyGrid = zeros(0,1);
   end
   
   properties (Dependent)
      % HINFSYN: Automatic plant scaling [{'on'} | 'off'].
      AutoScale
      % HINFSYN: Automatic regularization [{'on'} | 'off'].
      Regularize
      % HINFSYN: Limit controller gains [{'on'} | 'off']
      LimitGain
      
      % HINFSTRUCT: Number of randomized starts (default = 0).
      RandomStart
      % HINFSTRUCT: Parallel processing flag (default = false)
      UseParallel
      % HINFSTRUCT: Minimum decay rate for closed-loop poles (default = 1e-7)
      MinDecay
      % HINFSTRUCT: Maximum natural frequency of closed-loop poles (default=Inf).
      MaxFrequency
   end
   
   properties (Hidden)
      % Temporary option to change smoothing of D/G scales
      % ['off' | {'incremental'} | 'mincond']
      % XXX Set default as 'off' for initial testing
      Smoothing = 'incremental'; % 'off';
      % HINFSYN options
      hinfsynOPT
      % HINFSTRUCT options
      hinfstructOPT
   end
   
   properties (Constant,Hidden)
      % Accuracy of MU computation (see DGLMIsys)
      TolMU = 5e-3;
   end
   
   
   methods
      % Constructor
      function opt = musyn
         % Create option sets for HINFSYN and HINFSTRUCT
         opt.hinfsynOPT = hinfsynOptions;
         opt.hinfstructOPT = hinfstructOptions;
      end
      
      function op = set.MaxIter(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && rem(V,1)==0 && V>0 && V<Inf
            op.MaxIter = double(V);
         else
            error(message('Robust:analysis:PositiveInteger','MaxIter'))
         end
      end
      
      function op = set.TargetPerf(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0 && V<Inf
            op.TargetPerf = double(V);
         else
            error(message('Robust:design:NonNegativeScalar','TargetPerf'))
         end
      end
      
      function op = set.TolPerf(op,V)
         % Allow TolPerf = 0.  This setting is used to turn off
         % the relative tolerance stopping condition.
         if isnumeric(V) && isscalar(V) && isreal(V) && V>=0 && V<Inf
            op.TolPerf = double(V);
         else
            error(message('Robust:design:NonNegativeScalar','TolPerf'))
         end
      end
      
      function op = set.Display(op,V)
         V = ltipack.matchKey(V,{'off','short','full'});
         if isempty(V)
            error(message('Robust:design:musyn1'))
         else
            op.Display = V;
         end
      end
      
      function op = set.MixedMU(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','MixedMU'))
         else
            op.MixedMU = V;
         end
      end
      
      function op = set.FullDG(op,V)
         if (isnumeric(V) && any(numel(V)==[1 2]) && all(V==0 | V==1))
            V = logical(V);
         elseif ~(islogical(V) && any(numel(V)==[1 2]))
            error(message('Robust:design:musyn15'))
         end
         if isscalar(V)
            op.FullDG = V(:,[1 1]);
         else
            op.FullDG = reshape(V,[1 2]);
         end
      end
      
      function op = set.FitOrder(op,V)
         if ~(isnumeric(V) && isreal(V) && any(numel(V)==[1 2]) && all(V>=0 & rem(V,1)==0))
            error(message('Robust:design:musyn3'))
         else
            V = double(V);
            if isscalar(V)
               op.FitOrder = V(:,[1 1]);
            else
               op.FitOrder = reshape(V,[1 2]);
            end
         end
      end
      
      function op = set.FrequencyGrid(op,V)
         if isnumeric(V) && isreal(V) && (isempty(V) || isvector(V)) && all(V>=0)
            op.FrequencyGrid = sort(double(V(:)));
         else
            error(message('Robust:design:musyn2'))
         end
      end
            
      % HINFSYN OPTIONS
      function V = get.AutoScale(op)
         V = op.hinfsynOPT.AutoScale;
      end
      
      function op = set.AutoScale(op,V)
         try
            op.hinfsynOPT.AutoScale = V;
         catch ME
            throw(ME)
         end
      end
      
      function V = get.Regularize(op)
         V = op.hinfsynOPT.Regularize;
      end
      
      function op = set.Regularize(op,V)
         try
            op.hinfsynOPT.Regularize = V;
         catch ME
            throw(ME)
         end
      end
      
      function V = get.LimitGain(op)
         V = op.hinfsynOPT.LimitGain;
      end
      
      function op = set.LimitGain(op,V)
         try
            op.hinfsynOPT.LimitGain = V;
         catch ME
            throw(ME)
         end
      end
      
      % HINFSTRUCT OPTIONS
      function V = get.RandomStart(op)
         V = op.hinfstructOPT.RandomStart;
      end
      
      function op = set.RandomStart(op,V)
         try
            op.hinfstructOPT.RandomStart = V;
         catch ME
            throw(ME)
         end
      end
      
      function V = get.UseParallel(op)
         V = op.hinfstructOPT.UseParallel;
      end
      
      function op = set.UseParallel(op,V)
         try
            op.hinfstructOPT.UseParallel = V;
         catch ME
            throw(ME)
         end
      end
      
      function V = get.MinDecay(op)
         V = op.hinfstructOPT.MinDecay;
      end
      
      function op = set.MinDecay(op,V)
         try
            op.hinfstructOPT.MinDecay = V;
         catch ME
            throw(ME)
         end
      end
      
      function V = get.MaxFrequency(op)
         V = op.hinfstructOPT.MaxFrequency;
      end
      
      function op = set.MaxFrequency(op,V)
         try
            op.hinfstructOPT.MaxFrequency = V;
         catch ME
            throw(ME)
         end
      end
             
%       % Get methods for dependent properties
%       function out = get.FullDisplay(op)
%          out = strcmpi(op.Display,'full') || strcmpi(op.Mode,'manual');
%       end
%       
%       function out = get.CompactDisplay(op)
%          out = ~strcmpi(op.Display,'off') || strcmpi(op.Mode,'manual');
%       end
      
      % Set methods for hidden Smoothing property
      function op = set.Smoothing(op,V)
         V = ltipack.matchKey(V,{'off','incremental','mincond'});
         if isempty(V)
            error(message('Robust:analysis:OnOffOption','Smoothing'))
         else
            op.Smoothing = V;
         end
      end
      
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'musyn';
      end
   end
   
end
