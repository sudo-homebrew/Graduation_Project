classdef ucomplexm < UncertainBlock & StaticModel
%UCOMPLEXM  Creates uncertain complex matrix.
%
%   A = UCOMPLEXM(NAME,NOMINAL) creates an uncertain complex-valued matrix A
%   taking values in a unit ball centered at the nominal value NOMINAL. More 
%   precisely,
%      A = NOMINAL + DELTA   
%   where DELTA can be any complex-valued matrix satisfying norm(DELTA) <= 1.
%   The string NAME specifies the uncertain matrix name. The resulting object A 
%   is of class @ucomplexm.
% 
%   A = UCOMPLEXM(NAME,NOMINAL,'WL',WL,'WR',WR) creates an uncertain complex-
%   valued matrix of the form
%       A = NOMINAL + WL * DELTA * WR
%   where DELTA can be any complex-valued matrix satisfying norm(DELTA) <= 1.
%   The weigthing matrices WL and WR must be invertible and their default value
%   are identity matrices.
%
%   Use the "AutoSimplify" property to control how expressions involving
%   uncertain blocks are simplified, type "help ucomplexm.AutoSimplify" for
%   details.
%
%   Example:
%      % Create an uncertain complex matrix "F" with nominal value [1 2 3i;0 5 6] 
%      % and weighting matrices WL = diag([.1 .3]) and WR = diag([.4 .8 1.2]):
%      F = ucomplexm('F',[1 2 3i;0 5 6],...
%                    'WL',diag([.1 .3]),...
%                    'WR',diag([.4 .8 1.2]));
%
%   See also UREAL, UCOMPLEX, ULTIDYN, UMAT, UNCERTAINBLOCK.
   
%   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2012 The MathWorks, Inc.
      
   properties (Access = public, Dependent)   
      % Nominal value (scalar).
      NominalValue
      % Left weigthing matrix.
      WL
      % Right weigthing matrix.
      WR
   end
   
   properties (Access = protected)
      % Storage properties (uncoupled)
      WL_;
      WR_;
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
      
      function blk = ucomplexm(name,value,varargin)
         ni = nargin;
         if rem(ni,2)~=0
            ctrlMsgUtils.error('Robust:umodel:ucomplexm1')
         elseif ni==0
            name = 'UNNAMED';  value = 0;
         end
         [ny,nu] = size(value);
         blk.IOSize_ = [ny nu];
         try
            % Use SET functions to validate NAME and VALUE arguments
            blk.Name = name;
            blk.NominalValue = value;
            % Process additional inputs.
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

      function Value = get.WL(blk)
         % GET method for WL property
         Value = blk.WL_;
         if isempty(Value)
            Value = eye(blk.IOSize_(1));
         end
      end
      
      function Value = get.WR(blk)
         % GET method for WR property
         Value = blk.WR_;
         if isempty(Value)
            Value = eye(blk.IOSize_(2));
         end
      end
                           
      function blk = set.NominalValue(blk,Value)
         % SET method for NominalValue property
         if ~(isnumeric(Value) && isequal(size(Value),blk.IOSize_) && ~hasInfNaN(Value))
            ctrlMsgUtils.error('Robust:umodel:ucomplexm2')
         end
         blk.NominalValue_ = full(double(Value));
      end
      
      function blk = set.WL(blk,Value)
         % SET method for WL property
         blk.WL_ = localValidateWeight(Value,'WL',blk.IOSize_(1));
      end
      
      function blk = set.WR(blk,Value)
         % SET method for WR property
         blk.WR_ = localValidateWeight(Value,'WR',blk.IOSize_(2));
      end
            
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      function displaySize(~,~)
         % Display for "size(M)"
         ios = blk.IOSize_;
         disp(ctrlMsgUtils.message('Robust:umodel:SizeUCOMPLEX',ios(1),ios(2)))
      end
      
      function blk = uscale_(blk,fact)
         % Scales normalized uncertainty level
         if isempty(blk.WL_)
            ny = blk.IOSize_(1);
            blk.WL_ = diag(repmat(fact,[ny 1]));
         else
            blk.WL_ = fact * blk.WL_;
         end
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
         Offset = blk.NominalValue_;
      end
      
      function D = ltipack_ssdata(blk,varargin)
         % Converts to ltipack.ssdata object
         d = numeric_array(blk,varargin{:});
         ios = blk.IOSize_;
         D = ltipack.ssdata([],zeros(0,ios(2)),zeros(ios(1),0),d,[],0);
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
            if ~isempty(R)
               M = lft(R,M);
            end
         end
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         str = ctrlMsgUtils.message('Robust:umodel:ucomplexm6',...
            blk.Name,sprintf('%dx%d',blk.IOSize_),ncopies);
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
         WL = blk.WL_; %#ok<*PROP>
         WR = blk.WR_;
         [ny,nu] = size(S);
         noWeight = [isempty(WL) isempty(WR)];
         if noWeight(1)
            WL = eye(ny);  WLi = WL;
         else
            WLi = inv(WL);
         end
         if noWeight(2)
            WR = eye(nu);  WRi = WR;
         else
            WRi = inv(WR);
         end
         T = [S WL;WR zeros(nu,ny)];
         if all(noWeight)
            % Already normalized
            R = [];
         else
            R = [zeros(ny,nu) WLi;WRi zeros(nu,ny)];
         end
      end
      
      function Aval = norm2act(blk,Nval)
         % Converts normalized values to actual block values. The transformation
         % is A = S + WL * N * WR where S is the nominal value. 
         if ~isnumeric(Nval)
            ctrlMsgUtils.error('Robust:umodel:norm2act3')
         end
         sN = size(Nval);
         if ~isequal(sN(1:2),blk.IOSize_)
            ctrlMsgUtils.error('Robust:umodel:norm2act4',blk.IOSize_(1),blk.IOSize_(2))
         end
         Aval = zeros(sN);
         Nval = double(Nval);
         S = blk.NominalValue_;  WL = blk.WL;  WR = blk.WR; %#ok<*PROPLC>
         for ct=1:prod(sN(3:end))
            Aval(:,:,ct) = S + WL * Nval(:,:,ct) * WR;
         end
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values. The 
         % transformation is N = WL\(A-S)*WR where S is the nominal value.
         if ~isnumeric(Aval)
            ctrlMsgUtils.error('Robust:umodel:act2norm3')
         end
         sA = size(Aval);
         if ~isequal(sA(1:2),blk.IOSize_)
            ctrlMsgUtils.error('Robust:umodel:act2norm4',blk.IOSize_(1),blk.IOSize_(2))
         end
         nv = prod(sA(3:end));
         Nval = zeros(sA);
         Aval = double(Aval);
         S = blk.NominalValue_;
         if nv==1
            Nval = blk.WL \ (Aval-S) / blk.WR;
         elseif nv>1
            [L1,U1,p1] = lu(blk.WL,'vector');
            [L2,U2,p2] = lu(blk.WR,'vector');
            S = S(p1,:);
            for ct=1:nv
               Nval(:,p2,ct) = U1 \ (L1 \ (Aval(p1,:,ct)-S) / U2) / L2;
            end
         end
         if nargout>1
            Ndist = zeros([sA(3:end) 1 1]);
            for ct=1:nv
               Ndist(ct) = norm(Nval(:,:,ct));
            end
         end
      end
      
      function CS = randSample_(blk,N)
         % Randomly samples uncertain complex matrix. Random values are
         % generated for the normalized block and transformed to actual 
         % values. Returns N-by-1 cell array of matrix values.
         S = blk.NominalValue_;  WL = blk.WL;  WR = blk.WR;
         SAS = [blk.IOSize_ , N];
         SampleArray = complex(rand(SAS)-0.5,rand(SAS)-0.5);
         for ct=1:N
            delta = SampleArray(:,:,ct);
            sf = sqrt(rand(1)) / norm(delta);  % check...
            SampleArray(:,:,ct) = S + WL * (sf * delta) * WR;
         end
         CS = squeeze(num2cell(SampleArray,[1 2]));
      end
      
      function blk = getNormalizedForm(blk)
         % Returns normalized version of block
         blk = ucomplexm(sprintf('%sNormalized',blk.Name),zeros(blk.IOSize_));
      end
            
   end
   
   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
   end
   
end

%--------------------------------------------
function Value = localValidateWeight(Value,Wstr,N)
% Validates values for WL and WR
if ~(isnumeric(Value) && isequal(size(Value),[N N]) && ~hasInfNaN(Value))
   ctrlMsgUtils.error('Robust:umodel:ucomplexm3',Wstr,N)
elseif rank(Value)<N
   ctrlMsgUtils.error('Robust:umodel:ucomplexm4',Wstr)
end
Value = full(double(Value));
if isequal(Value,eye(N))
   Value = [];  % compact storage
end
end
