classdef (Hidden) IMMSmoother < matlabshared.smoothers.internal.AbstractSmoother ...
        & matlabshared.tracking.internal.AbstractContainsFilters

    % This is an internal class and may be removed or modified in a future
    % release.
    %
    % This class provides the smoother for IMM filter. This smoother class
    % supports only supports Gaussian filters with Smooth Methods. Inherit
    % from this class to implement a smoothing algorithm for an IMM filter.
    % The IMMSmoother class assumes that the filter is divided into 2 main
    % steps - prediction and correction. 
    % 
    % Inheritance will give acccess to 2 new properties:
    %
    % EnableSmoothing
    % MaxNumSmoothingSteps
    %   
    % You can use the following workflow to parse name/value pairs for the
    % Smoother
    % 
    % [smootherArgs, idx1, idx2] = yourClassName.parseSmoothingNVPairs(varargin);
    % filterArgs = {varargin{1:idx1-1},varargin{idx1 + 2:idx2 - 1},varargin{idx2 + 2:end}};
    % 
    % The parent class must use the following workflow.
    % 
    % In the prediction method:
    %
    % function yourPredictMethodName(obj, varargin)
    %   setupInitialDistributions(obj);
    % 
    %   % your custom prediction code goes here.
    %   % ----
    %   % your custom prediction code ends here.
    %
    %   obj.LastStepTime(1) = time step for the current predict call.
    %   updatePredictionData(obj)
    % end
    %
    % In the correction method:
    % 
    %  function yourCorrectMethodName(obj, varargin)
    % 
    %   % your custom correction code goes here.
    %   % ----
    %   % your custom correction code ends here.
    %   
    %   updateCorrectionData(obj)
    % end
    %
    % At any time step, you can call smooth method on your object with the
    % following syntax.
    % smooth(obj);
    % smooth(obj, numSteps); where numSteps <= min(numPredictCalls,MaxNumSmoothingSteps);
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties (Access = protected, Abstract)
        pTrackingFilters
        
        pModelProbabilities
        
        pTransitionProbabilities
        
        pNumModels
    end
    
    properties (Access = public, Abstract)
        %ModelConversionFcn convert State or StateCovariance
        %   Specify a function handle to the Model conversion
        %   function. Which converts from (i-th) filter to (j-th) filter.
        %
        %   Default: @switchimm
        ModelConversionFcn
    end
    
    properties(Access = protected)
        % LastStepTime has the last predict time step value
        LastStepTime;
    end
    
    methods
        function obj = IMMSmoother(varargin)
            obj@matlabshared.smoothers.internal.AbstractSmoother(varargin{:});
        end
    end
    % Override smooth to provide IMM smoother help text
    methods
        function varargout = smooth(obj, varargin)
            % [x, P, w] = smooth(obj) runs a backward recursion to obtain
            % smoothened states at previous steps. The number of backward
            % steps are governed based on the number of executed forward
            % steps (F) and the maximum number of backward steps (B)
            % specified by MaxNumBackwardSteps property. A filter moves
            % forward whenever predict is called. If F < B, number of
            % backward steps is F - 1; Otherwise number of backward steps
            % is B.
            %
            % x is a matrix with number of columns equal to the number of
            % backward steps. Each column represents the state at previous
            % steps. The first column represents the state at the end of
            % backward recursion. The last column represents the state at
            % the beginning of backward recursion.
            %
            % P is a 3-dimensional matrix with number of pages equal to the
            % number of backward steps. Each page represents the error
            % covariance associated with the estimate provided by the
            % output, x.
            %
            % w is a matrix with number of columns equal to the number of
            % backward steps. Each column represents the smoothened model
            % probabilities used in calculating x and P.
            %
            % [x, P, w] = smooth(obj, numBackwardSteps) allows you to
            % specify the number of backward steps. numBackwardSteps must
            % be less than the maximum number of permissible steps.
            %
            [varargout{1:nargout}] = smooth@matlabshared.smoothers.internal.AbstractSmoother(obj,varargin{:});
        end
    end
%     
   methods (Access = protected)        
       function dist = distribution(obj)
           % Definition of distribution of the filter.
           N = obj.pNumModels;
           filterDists = cell(N,1);
           for j = coder.unroll(1:N)
               % trackingKF.modelName outputs a varsize, non-constant
               % output. This will confuse switchimm's validation if motion
               % model is undefined but state is fixed-size.
               % Therefore, using a variable-sized distribution for KF.
               if isa(obj.pTrackingFilters{j},'trackingKF')
                   filterDists{j} = struct('State',obj.pTrackingFilters{j}.State,'StateCovariance',obj.pTrackingFilters{j}.StateCovariance);
               else
                   filterDists{j} = distribution(obj.pTrackingFilters{j});
               end
           end       
           % Store Model Prob
           w = obj.pModelProbabilities(:);
           
           dist = struct('ModelProbabilities',w,'FilterDistributions',{filterDists});
       end
       
       function d = stateTransitionData(obj)
           % Describe the state Transition Data distribution for the IMM
           % filter
           N = obj.pNumModels;
           filterStData = cell(N,1);
           for i = coder.unroll(1:N)
               filterStData{i} = stateTransitionData(obj.pTrackingFilters{i});
           end
           d = struct('dT',obj.LastStepTime,...
               'TransitionProbabilities',obj.pTransitionProbabilities,...
               'FilterStateTransitionData',{filterStData});
       end
       
       function ensureMethodDefinition(obj)
           % IMMSmoother's methods are defined if LastStepTime is defined
           % and each filter's methods are defined.
           ensureLastStepTimeIsDefined(obj)
           coder.unroll()
           for i = 1:obj.pNumModels
               if isa(obj.pTrackingFilters{i},'matlabshared.smoothers.internal.GaussianSmoother')
                   ensureMethodDefinition(obj.pTrackingFilters{i});
               end
           end
       end
       
       function ensureLastStepTimeIsDefined(obj)
           if ~obj.pIsInitialized
               if ~coder.internal.is_defined(obj.LastStepTime)
                   % LastStepTime will have the same data type as IMM state
                   % If the dt differs from obj.State data type it will be
                   % casted to data type of obj.LastStepTime
                   obj.LastStepTime = zeros(1,1,'like',obj.State);
               end
           else
               coder.assertDefined(obj.LastStepTime);
           end
       end
       
       function s = smoothOneStep(obj,s,c,p,d)         
           % Describe the smoothing recursion for IMM Filter
           
           % Filter smooth likelihoods
           filterLikelihoods = calculateSmoothLikelihood(obj,s,p,d);

           % Smooth distribution mixing before each filter Smooth
           s = mixModelsSmooth(obj,s,c,p,d);
           
           % Filter Smoothing 
           s = smoothFilterDists(obj,s,c,p,d);
           
           % Update Smooth Model Probabilities 
           wt = c.ModelProbabilities;
           wts = bsxfun(@times,wt,filterLikelihoods);
           s.ModelProbabilities = wts/sum(wts);
       end
       
       function filterLikelihoods = calculateSmoothLikelihood(obj,s,p,d)
           % filterLikelihoods = calculateSmoothLikelihood(obj,s,p) calculates
           % each filters smooth state likelihood based on k+1 step smooth
           % distribution, s, and k+1 step predict distribution, p
           modelNames = cell(obj.pNumModels,1);
           for i = coder.unroll(1:obj.pNumModels)
               modelNames{i} = obj.pTrackingFilters{i}.modelName;
           end
           transProb = real(d.TransitionProbabilities^complex(d.dT));
           filterLikelihoods = zeros(obj.pNumModels,1,'like',obj.State);
           for i = coder.unroll(1:obj.pNumModels)
               Xi = s.FilterDistributions{i}.State;
               Xzeros = zeros(size(Xi),'like',Xi);
               for kMdl = coder.unroll(1:obj.pNumModels)
                   Xk = s.FilterDistributions{kMdl}.State;
                   Xk = obj.ModelConversionFcn(modelNames{kMdl},Xk,modelNames{i},Xzeros);
                   % convert the StateCovariance too for the correct Size
                   filterSmoothDist = struct('State',Xk, ...
                       'StateCovariance',s.FilterDistributions{i}.StateCovariance);
                   filterPredictDist = p.FilterDistributions{i};
                   filterLikelihoods(i) = filterLikelihoods(i) + transProb(i,kMdl)*smoothLikelihood(obj.pTrackingFilters{i},filterSmoothDist,filterPredictDist);
               end
           end
       end
       
       function sSmooth = smoothFilterDists(obj,s,c,p,d)
           % sSmooth = smoothFilterDists(obj,s,c,p,d) runs the smooth step
           % for each of the filter and update the smooth distribution for
           % them, the updated smooth distribution is returned as function
           % output
           % s = smooth distribution at step k+1
           % c = correct distribution at step k
           % p = predict distribution at step k+1
           % d = state transition distribution from step k to k+1
           sSmooth = s;
           for i = coder.unroll(1:obj.pNumModels)
               filterSmoothDist = s.FilterDistributions{i};
               filterPredictDist = p.FilterDistributions{i};
               filterCorrectDist = c.FilterDistributions{i};
               filterStData = d.FilterStateTransitionData{i};
               newFilterSmoothDist = smoothOneStep(obj.pTrackingFilters{i}, filterSmoothDist, filterCorrectDist, filterPredictDist, filterStData);
               sSmooth.FilterDistributions{i} = newFilterSmoothDist;
           end
       end
       
       function s = genDist(obj)
           % Create Distribution when called for trackingIMM
           % Note: The distribution(obj) function cannot be used because PF
           % and other non-supportred filters doesn't have distribution
           % function defined
           N = obj.pNumModels;
           filterDists = cell(N,1);
           for j = coder.unroll(1:N)
               filterDists{j} = struct('State',obj.pTrackingFilters{j}.State, ...
                   'StateCovariance',obj.pTrackingFilters{j}.StateCovariance);
           end
           % Store Model Prob
           w = obj.pModelProbabilities(:);
           
           s = struct('ModelProbabilities',w,'FilterDistributions',{filterDists});
       end
       
       function [x, P] = combineDist(obj,s)
           % [x, P] = combineDist combines the distribution of all filter
           % into single state and state covariance using model
           % probabilities
           if nargin == 1
               s = genDist(obj);
           end
           
           modelProb = s.ModelProbabilities;
           mdlOut = 1; % First model defines state for output
           
           % Checking first model state is a column vector or not
           cond = iscolumn(obj.pTrackingFilters{mdlOut}.State);
           if cond
               Xout = obj.pTrackingFilters{mdlOut}.State;
           else
               Xout = obj.pTrackingFilters{mdlOut}.State';
           end
           Xout = zeros(size(Xout),'like',Xout);
           Xzeros = Xout;
           modelNames = cell(obj.pNumModels,1);
           for i = coder.unroll(1:obj.pNumModels)
               modelNames{i} = obj.pTrackingFilters{i}.modelName;
           end

           for kMdl = coder.unroll(1:obj.pNumModels)
               Xk = s.FilterDistributions{kMdl}.State;
               Xk = obj.ModelConversionFcn(modelNames{kMdl},Xk,modelNames{mdlOut},Xzeros);
               % Combine k output
               Wk = modelProb(kMdl);
               
               Xout = Xout + Wk*Xk;
           end
           if cond
               x = Xout;
           else
               x = Xout';
           end
           
           Pout = obj.pTrackingFilters{mdlOut}.StateCovariance;
           Pout = zeros(size(Pout),'like',Pout);
           Pzeros = Pout;
           for kMdl = coder.unroll(1:obj.pNumModels)
               
               Xk = s.FilterDistributions{kMdl}.State;
               Pk = s.FilterDistributions{kMdl}.StateCovariance;
               
               % Convert model k to output model
               Xk = obj.ModelConversionFcn(modelNames{kMdl},Xk,modelNames{mdlOut},Xzeros);
               Pk = obj.ModelConversionFcn(modelNames{kMdl},Pk,modelNames{mdlOut},Pzeros);
               % Combine k output
               Wk = modelProb(kMdl);
               
               res = Xk-Xout;
               
               Pout = Pout + Wk*(Pk+res*res');
           end
           P = Pout;
       end

       function [x, P, w] = sendToOutput(obj,smoothDists)
           % [x, P, w] = sendToOutput(obj, smoothDists) output an array of
           % distributions as concatenated states, state covariances and
           % smoothened model probabilities using smoothDists (collection
           % of smoothened distributions)
           
           numSteps = numel(smoothDists);
           numStates = numel(obj.State);
           x = zeros(numStates,numSteps,'like',obj.State);
           P = zeros(numStates,numStates,numSteps,'like',obj.StateCovariance);
           w = zeros(obj.pNumModels,numSteps,'like',obj.ModelProbabilities);
           
           for i = 1:numSteps
               w(:,i) = smoothDists(i).ModelProbabilities;
               % combining smooth distributions of each filter into smooth
               % state and state covariance
               [x(:,i), P(:,:,i)] = combineDist(obj,smoothDists(i));
           end
       end
       
       function s = mixModelsSmooth(obj,s,c,p,d)
           % Update the smooth distribution of each filter by mixing
           % Corrected model probabilities at time t
           wtt = c.ModelProbabilities;
           wtplusk = s.ModelProbabilities;
           % Transpose because bij is using pji
           pji = max(0,(real((d.TransitionProbabilities)^complex(d.dT)))');
           
           % backward transition probability
           bij = bsxfun(@times,pji,wtt');
           
           bij = bsxfun(@rdivide,bij,sum(bij,2)); 
           
           % Fixing case where the forward model prob == 0
           iFnd = find(wtt == 0);
           for p = 1:numel(iFnd)
               bij(iFnd(p),:) = 0;
               bij(iFnd(p),iFnd(p)) = 1;
           end
           
           % Backward mixing probability
           mixProb = bsxfun(@times,bij,wtplusk);
           mixProb = bsxfun(@rdivide,mixProb,sum(mixProb,1));
           
           % Fixing case where the smooth model prob == 0
           iFnd = find(wtplusk == 0);
           for p = 1:numel(iFnd)
               mixProb(:,iFnd(p)) = 0;
               mixProb(iFnd(p),iFnd(p)) = 1;
           end
           s = mixDistribution(obj,mixProb,s);
       end
       
       function s = mixDistribution(obj,mixProb,s)
           % calculateMixModel(obj,mixProb,s) uses mixProb to mix state and
           % stateCovariances and outputs them in the form of distribution
           % Note: the obj is not updated with new mixed state
           % and stateCovariance and if needed s can be used to update it
           
           if nargin == 2
               % When called from trackingIMM
               s = genDist(obj);
           end
           % Mix States
           Xorg = cell(obj.pNumModels,1);
           Porg = cell(obj.pNumModels,1);
           modelNames = cell(obj.pNumModels,1);

           for i = coder.unroll(1:obj.pNumModels)
               modelNames{i} = obj.pTrackingFilters{i}.modelName;
               Xorg{i} = s.FilterDistributions{i}.State; % Save for mixing
               Porg{i} = s.FilterDistributions{i}.StateCovariance; % Save for mixing
           end
           % Mix state
           for jMdl = coder.unroll(1:obj.pNumModels)
               Xj = Xorg{jMdl};
               Xj = zeros(size(Xj),'like',Xj); 
               for kMdl = coder.unroll(1:obj.pNumModels)
                   Xk = Xorg{kMdl};
                   
                   % Convert model k to model j
                   Xkj = obj.ModelConversionFcn(modelNames{kMdl},Xk,modelNames{jMdl},Xj);
                   
                   % Mix k into j
                   Wkj = mixProb(kMdl, jMdl);
                   
                   Xj = Xj + Wkj*Xkj;
               end
               s.FilterDistributions{jMdl}.State = Xj;
           end
       
           % Mix state covariances
           for jMdl = coder.unroll(1:obj.pNumModels)
               Xj = s.FilterDistributions{jMdl}.State;
               Pj = Porg{jMdl};
               Pj = zeros(size(Pj),'like',Pj);
               
               for kMdl = coder.unroll(1:obj.pNumModels)
                   Xk = Xorg{kMdl};
                   Pk = Porg{kMdl};
                   
                   % Convert model k to model j
                   Xk = obj.ModelConversionFcn(modelNames{kMdl},Xk,modelNames{jMdl},Xj);
                   Pk = obj.ModelConversionFcn(modelNames{kMdl},Pk,modelNames{jMdl},Pj);
                   
                   % Mix k into j
                   Wkj = mixProb(kMdl, jMdl);
                   
                   res = Xk-Xj;
                   
                   Pj = Pj + Wkj*(Pk + res*res');
               end
               s.FilterDistributions{jMdl}.StateCovariance = Pj;
           end
       end
       function currDist = fixFilterPredictDist(obj)
           % currDist = fixFilterPredictDist(obj) fixes the stored predict
           % distribution needed for smoothing.
           % The predict distribution stored by filter is based on
           % prediction on mix states, however the smoother needs
           % prediction based on previous correct values without mixing.
           % Therefore the values are synced with previous correct step,
           % prediction is done again and then values are stored for
           % smoothing
           
           idx = obj.CurrentIndex;
           N = obj.pNumModels;
           currDist = cell(N,1);
           correctDist = getCorrectedDistributionAtStep(obj,idx);
           for j = coder.unroll(1:N)
               filterj = obj.pTrackingFilters{j};
               
               % Storing filter current states based on mixing after
               % predict
               currDist{j} = distribution(filterj);
               
               % Setting filters to their corrected distribution
               syncDist(filterj,correctDist.FilterDistributions{j});
               
               % accessing dT from filter as the current dt is not yet
               % updated by stateTransitionData(obj)
               dT = obj.LastStepTime;
               
               % disabling storage for individual filters
               % Applicable in scenario when individual filters have
               % EnableSmoothing == true
               if (filterj.EnableSmoothing)
                   disableStorage(filterj);
               end
               
               % predict values without mixing
               predict(filterj,dT);
               
               % enabling storage again
               if (filterj.EnableSmoothing)
                   enableStorage(filterj);
               end
               
           end
       end
       function revertFilterPredictDist(obj,currDist)
           % Revert back to old state and state covariance for each filter
           % after fixing the predictDist
           N = obj.pNumModels;
           for j = coder.unroll(1:N)
               filterj = obj.pTrackingFilters{j};
               syncDist(filterj,currDist{j});
           end    
       end
   end
       %% save/load/clone helpers
       methods (Access = protected)
           function copySmootherProperties(obj,obj2)
               if coder.internal.is_defined(obj.LastStepTime)
                   obj2.LastStepTime = obj.LastStepTime;
               end
               copySmootherProperties@matlabshared.smoothers.internal.AbstractSmoother(obj, obj2);
           end
           
           function s = saveSmootherProperties(obj, varargin)
               s = saveSmootherProperties@matlabshared.smoothers.internal.AbstractSmoother(obj, varargin{:});
               s.LastStepTime = obj.LastStepTime;
           end
           
           function loadSmootherProperties(obj, s)
               loadSmootherProperties@matlabshared.smoothers.internal.AbstractSmoother(obj, s);
               if isfield(s,'LastStepTime')
                   obj.LastStepTime = s.LastStepTime;
               end
           end
   end

   methods (Access = {?matlab.unittest.TestCase,?matlabshared.smoothers.internal.AbstractSmoother})
       function updatePredictionData(obj)
           % The predict distribution stored by filter is based on
           % prediction on mixed states, however the smoother needs
           % prediction based on previous correct values without mixing.
           % Therefore the updatePredictData(obj) function is modified
           % Predict states are synced with previous correct step,
           % prediction is done again and then new values are stored for
           % smoothing
           if obj.EnableSmoothing
               % getting current distribution of the filter and replacing
               % it with the fixed distribution for the storage
               currDist = fixFilterPredictDist(obj);
               
               % storage of the fixed distribution
               updatePredictionData@matlabshared.smoothers.internal.AbstractSmoother(obj);
               
               % reverting each filter to their previous distributions
               revertFilterPredictDist(obj,currDist)
           end
       end

   end
   methods(Static,Hidden)
       function props = matlabCodegenNontunableProperties(~)
           % Let the coder know about non-tunable parameters so that it
           % can generate more efficient code.
           propsSmoother = matlabshared.smoothers.internal.AbstractSmoother.matlabCodegenNontunableProperties;
           props = {propsSmoother{:},'pNumModels'};
       end
   end
   
end