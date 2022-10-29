classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) dksyn < ltioptions.Generic
   % Options set for RobustStab Calculations
   
%   Copyright 2010-2012 The MathWorks, Inc.
   properties (Dependent)
   end
   
   properties
      % Optional frequency vector.
      FrequencyVector = zeros(0,1);
      % Optional controller used to initiate iteration.
      InitialController = [];
      % Automated mu-synthesis mode [{'on'} | 'off'].
      AutoIter = 'on';
      % Displays iteration progress in AutoIter mode [{'off'} | 'on']
      DisplayWhileAutoIter = 'off';
      % Starting iteration number.
      StartingIterationNumber = 1;
      % Number of iterations to perform.
      NumberOfAutoIterations = 10;
      % Real/complex mu-synthesis [{'off'} | 'on']
      MixedMU = 'off';
      % State order for fitting D-scaling and G-scaling data.
      AutoScalingOrder = [5 2];
      % Automatic termination of iteration procedure [{'on'} | 'off'].
      AutoIterSmartTerminate = 'on';
      % Tolerance for AutoIterSmartTerminate mode.
      AutoIterSmartTerminateTol = 0.005;
   end
   
   
   methods
      
      function op = set.FrequencyVector(op,V)
         if isnumeric(V) && isreal(V) && (isempty(V) || isvector(V)) && all(V>=0)
            op.FrequencyVector = sort(double(V(:)));
         else
            ctrlMsgUtils.error('Robust:design:FreqVector')
         end
      end
      
      function op = set.InitialController(op,V)
         if isempty(V)
            op.InitialController = [];
         else
            try
               V = ss.convert(V);
            catch ME
               error(message('Robust:design:InitialController'))
            end
            op.InitialController = V;
         end
      end
      
      function op = set.AutoIter(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','AutoIter');
         else
            op.AutoIter = V;
         end
      end
      
      function op = set.DisplayWhileAutoIter(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','DisplayWhileAutoIter');
         else
            op.DisplayWhileAutoIter = V;
         end
      end
      
      function op = set.StartingIterationNumber(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0 && rem(V,1)==0
            op.StartingIterationNumber = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:PositiveInteger','StartingIterationNumber');
         end
      end
      
      function op = set.NumberOfAutoIterations(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0 && rem(V,1)==0
            op.NumberOfAutoIterations = double(V);
         else
            ctrlMsgUtils.error('Robust:analysis:PositiveInteger','NumberOfAutoIterations')
         end
      end
      
      function op = set.MixedMU(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','MixedMU');
         else
            op.MixedMU = V;
         end
      end
      
      function op = set.AutoScalingOrder(op,V)
         if ~(isnumeric(V) && isreal(V) && any(numel(V)==[1 2]) && all(V>0 & rem(V,1)==0))
            ctrlMsgUtils.error('Robust:design:AutoScalingOrder')
         else
            V = double(V);
            if isscalar(V)
               op.AutoScalingOrder = V(:,[1 1]);
            else
               op.AutoScalingOrder = [V(1) V(2)];
            end
         end
      end
      
      function op = set.AutoIterSmartTerminate(op,V)
         V = ltipack.matchKey(V,{'off','on'});
         if isempty(V)
            ctrlMsgUtils.error('Robust:analysis:OnOffOption','AutoIterSmartTerminate')
         else
            op.AutoIterSmartTerminate = V;
         end
      end
      
      function op = set.AutoIterSmartTerminateTol(op,V)
         if isnumeric(V) && isscalar(V) && isreal(V) && V>0
            op.AutoIterSmartTerminateTol = double(V);
         else
            ctrlMsgUtils.error('Robust:design:PositiveScalar','AutoIterSmartTerminateTol')
         end
      end
       
   end
   
   methods (Access = protected)
      function cmd = getCommandName(~)
         cmd = 'dksyn';
      end
   end
   
end
