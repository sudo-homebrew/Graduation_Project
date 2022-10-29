classdef ucomplex < UncertainBlock & StaticModel
%UCOMPLEX  Creates uncertain complex scalar.
%
%   A = UCOMPLEX(NAME,NOMINAL) creates an uncertain complex-valued parameter
%   ranging in the disk of radius 1 centered at the value NOMINAL. The string 
%   NAME specifies the parameter name and the complex scalar NOMINAL specifies  
%   its nominal value. The resulting object A is of class "ucomplex".
% 
%   A = UCOMPLEX(NAME,NOMINAL,'Radius',R) specifies the maximum deviation R
%   from the nominal value (radius of the uncertainty disk). The default 
%   value is R=1.
%
%   A = UCOMPLEX(NAME,NOMINAL,'Percentage',P) specifies the maximum percentage
%   deviation P from the nominal value. The corresponding absolute deviation 
%   is R = (P/100) * abs(NOMINAL).
%
%   Use the "Radius" or "Percentage" property of A to query the absolute or
%   percentage uncertainty on A. In addition, use the "Mode" property to 
%   specify which uncertainty measure (radius or percentage) remains invariant 
%   when changing the nominal value. For example, if A.Mode='Percentage', 
%   then a change in nominal value has no effect on A.Percentage but changes 
%   A.Radius. Note that "Mode" is initialized based on how you specify the 
%   uncertainty level, for example, 
%      A = ucomplex('a',-2+4i,'Radius',2)
%   initializes A.Mode to 'Radius' while
%      A = ucomplex('a',-2+4i,'Percentage',5)
%   initializes A.Mode to 'Percentage'.
%
%   Use the "AutoSimplify" property to control how expressions involving
%   uncertain blocks are simplified, type "help ucomplex.AutoSimplify" for
%   details.
%
%   Example:
%      % Create a complex parameter 'delta' with 50% uncertainty around its 
%      % nominal value -2+3i: 
%      a = ucomplex('delta',-2+3i,'Percentage',50);
%
%      % The parameter delta ranges in a disk centered at -2+3i with radius 
%      % R = 0.5 * abs(-2+3i) ~ 1.803:
%      R = a.Radius
%
%   See also UREAL, UCOMPLEXM, ULTIDYN, UMAT, UNCERTAINBLOCK.
   
%   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2012 The MathWorks, Inc.
      
   properties (Access = public, Dependent)   
      % Nominal value (scalar).
      NominalValue
      % Uncertainty quantification mode.
      Mode
      % Maximum absolute deviation from nominal value.
      Radius
      % Maximum percentage deviation from nominal value. 
      Percentage
   end
   
   properties (Access = protected)
      % Storage properties (uncoupled)
      % Uncertainty quantification mode ([] or string)
      Mode_ = 'Radius';
      % Uncertainty level as absolute deviations from nominal value
      Radius_
   end
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function boo = isClosed(~)
         boo = false;
      end
      
      function T = toClosed(~)
         T = 'umat';
      end
      
      % Note: getAttributes never called (first converted to UMAT)      
   end
    
   %% PUBLIC METHODS
   methods
      
      function blk = ucomplex(name,value,varargin)
         ni = nargin;
         if rem(ni,2)~=0
            ctrlMsgUtils.error('Robust:umodel:ucomplex1')
         elseif ni==0
            name = 'UNNAMED';  value = 0;
         end
         blk.Radius_ = 1;
         blk.IOSize_ = [1 1];
         try
            blk.Name = name;
            blk.NominalValue = value;
            % Process additional inputs.
            % If MODE is not explicitly set, initialize it consistently
            % with how the uncertainty level is specified
            ltipack.mustBeNameValuePairs(varargin)
            PropNames = lower(string(varargin(1:2:ni-2)));
            if ~any(startsWith(PropNames,"m"))
               idx = find(startsWith(PropNames,"p") | startsWith(PropNames,"r"),1,'last');
               if ~isempty(idx)
                  varargin = [varargin , {'Mode'} varargin(2*idx-1)];
               end
            end
            if ~isempty(varargin)
               blk = set(blk,varargin{:});
            end
         catch ME
            throw(ME)
         end
      end
      
      function Value = get.NominalValue(blk)
         % GET method for NominalValue property
         Value = blk.NominalValue_;
      end

      function Value = get.Mode(blk)
         % GET method for Mode property
         Value = blk.Mode_;
      end
      
      function Value = get.Radius(blk)
         % GET method for Radius property
         Value = blk.Radius_;
      end
               
      function Value = get.Percentage(blk)
         % GET method for Percentage property
         Value = blk.Radius_ * (100/abs(blk.NominalValue_));
      end
            
      function blk = set.Mode(blk,Value)
         % SET method for Name property
         UMode = ltipack.matchKey(Value,{'Radius','Percentage'});
         if isempty(UMode)
            ctrlMsgUtils.error('Robust:umodel:ucomplex2')
         elseif blk.NominalValue_==0 && strcmp(UMode,'Percentage')
            ctrlMsgUtils.error('Robust:umodel:ublock2')
         else
            blk.Mode_ = UMode;
         end
      end
      
      function blk = set.NominalValue(blk,Value)
         % SET method for NominalValue property
         if ~(isnumeric(Value) && isscalar(Value) && isfinite(Value))
            ctrlMsgUtils.error('Robust:umodel:ucomplex4')
         end
         Value = full(double(Value));
         switch blk.Mode
            case 'Percentage'
               % Hold percentage constant
               if Value==0
                  ctrlMsgUtils.error('Robust:umodel:ublock3')
               end
               blk.Radius_ = blk.Radius_ * abs(Value/blk.NominalValue_);
         end
         blk.NominalValue_ = Value;
      end
      
      function blk = set.Radius(blk,Value)
         % SET method for Radius property
         blk.Radius_ = localValidateRange(Value,'Radius');
      end
      
      function blk = set.Percentage(blk,Value)
         % SET method for Percentage property
         if blk.NominalValue_==0
            ctrlMsgUtils.error('Robust:umodel:ublock4')
         end
         blk.Radius_ = localValidateRange(Value,'Percentage') * ...
            abs(blk.NominalValue_/100);
      end
      
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      function displaySize(~,~)
         % Display for "size(M)"
         disp(ctrlMsgUtils.message('Robust:umodel:SizeUCOMPLEX'))
      end
      
      function blk = uscale_(blk,fact)
         % Scales normalized uncertainty level
         blk.Radius_ = fact * blk.Radius_;
      end
      
   end
   
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
      
      %% MODEL CHARACTERISTICS
      function boo = isreal_(varargin)
         boo = false;
      end
      
      %% INDEXING
      function M = createLHS(~)
         % Creates LHS in assignment.
         % Returns 0x0 UMAT
         M = umat();
      end
      
      %% TRANSFORMATIONS
      function M = repmat_(blk,s)
         M = repmat_(umat(blk),s);
      end
      
      function M = uminus_(blk)
         M = uminus_(umat(blk));
      end
      
   end
   
   
   %% HIDDEN INTERFACES
   methods (Hidden)
      
      % CONTROLDESIGNBLOCK INTERFACE
      function Offset = getOffset(blk)
         % Default value is the nominal value
         Offset = blk.NominalValue_;  % REVISIT: real(nominal)?
      end
      
      function D = ltipack_ssdata(blk,varargin)
         % Converts to ltipack.ssdata object
         d = numeric_array(blk,varargin{:});
         D = ltipack.ssdata([],zeros(0,1),zeros(1,0),d,[],0);
      end
      
      function D = ltipack_frddata(blk,freq,varargin)
         % Converts to ltipack.frddata object
         d = numeric_array(blk,varargin{:});
         D = ltipack.frddata(repmat(d,[1 1 length(freq)]),freq,0);
      end
      
      function M = numeric_array(blk,R,S)
         % Converts to double scalar. When R,S are supplied,
         % computes blk-S if R=[] and lft(R,blk-S) otherwise.
         M = blk.NominalValue_;
         if nargin>1
            M = M-S;
            if ~(isempty(R) || M==0)
               M = R(1,1)+R(1,2)*R(2,1)*M/(1-M*R(2,2));
            end
         end
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         Nominal = num2str(blk.NominalValue_,'%.3g');
         switch blk.Mode_
            case 'Radius'
               str = ctrlMsgUtils.message('Robust:umodel:ucomplex8',...
                  blk.Name,Nominal,sprintf('%.3g',blk.Radius_),ncopies);
            case 'Percentage'
               str = ctrlMsgUtils.message('Robust:umodel:ucomplex9',...
                  blk.Name,Nominal,sprintf('%.3g',blk.Percentage),ncopies);
         end
      end
      
   end
   
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function M = subparen(blk,indices)
         % Indexing forces conversion to UMAT
         M = subparen(umat(blk),indices);
      end
      
   end
   
   
   %% UTILITIES
   methods (Hidden)
      
      function [R,S,T] = normalizeBlock(blk)
         % Computes R,S,T such that
         %    * LFT(R,blk-S) is normalized
         %    * blk = LFT(T,LFT(R,blk-S))
         % Returns R=[] if blk-S is already normalized (then blk = LFT(T,blk-S))
         S = blk.NominalValue_;
         rho = blk.Radius_;
         tau = sqrt(rho);
         T = [S tau;tau 0];
         if abs(rho-1)<100*eps
            % Already normalized
            R = [];
         else
            tau = 1/tau;
            R = [0 tau;tau 0];
         end
      end
      
      function Aval = norm2act(blk,Nval)
         % Converts normalized values to actual block values. The transformation
         % is A = S + RHO * N where S is the nominal value and RHO is the radius. 
         if ~isnumeric(Nval)
            ctrlMsgUtils.error('Robust:umodel:norm2act3')
         end
         Aval = blk.NominalValue_ + blk.Radius_ * double(Nval);
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values. The 
         % transformation is N = (A-S)/RHO where S is the nominal value 
         % and RHO is the radius. 
         if ~isnumeric(Aval)
            ctrlMsgUtils.error('Robust:umodel:act2norm3')
         end
         Nval = (double(Aval) - blk.NominalValue_) / blk.Radius_;
         if nargout>1
            Ndist = abs(Nval);
         end
      end
      
      function CS = randSample_(blk,N)
         % Randomly samples uncertain complex variable. Returns a cell array
         % of scalar double values.
         cv = complex(randn(N,1),randn(N,1));
         cv = sqrt(rand(N,1)) .* (cv ./ abs(cv));
         CS = num2cell(blk.NominalValue_ + blk.Radius_ * cv);
      end

      function blk = getNormalizedForm(blk)
         % Returns normalized version of block
         blk = ucomplex(sprintf('%sNormalized',blk.Name),0);
      end
            
   end
   
   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
   end
   
end

%--------------------------------------------
function Value = localValidateRange(Value,Prop)
% Validates values for Radius and Percentage
if ~(isnumeric(Value) && isreal(Value) && isscalar(Value) && all(isfinite(Value)) && Value>0)
   ctrlMsgUtils.error('Robust:umodel:ucomplex5',Prop)
end
Value = full(double(Value));
end
