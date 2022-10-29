classdef (InferiorClasses = {? matlab.graphics.axis.Axes, ? matlab.ui.control.UIAxes}) ultidyn < UncertainDynamics
   %ULTIDYN  Model uncertain linear time-invariant dynamics.
   %
   %  UH = ULTIDYN(NAME,IOSIZE) creates a block UH that represents arbitrary
   %  linear time-invariant (LTI) dynamics with frequency response gain no  
   %  larger than one. You can use this block to represent unmodeled dynamics  
   %  or quantify frequency response uncertainty. The string NAME and the 
   %  1-by-2 vector IOSIZE specify the block name and I/O size. For example, 
   %     dH = ultidyn('DeltaH',[2 3])
   %  models the uncertainty on a two-output, three-input system H(s) as 
   %  a maximum unit-gain deviation from the nominal frequency response.
   %  To model frequency-dependent uncertainty levels, multiply the ULTIDYN 
   %  block by a suitable "shaping" filter. For example,
   %     W = tf([1 .1],[.1 1])
   %     dH = W * dH
   %  specifies an uncertainty bound that increases from 0.1 at low frequencies
   %  to 10 at high frequencies.
   %
   %  UH = ULTIDYN(NAME,IOSIZE,'Bound',MAXGAIN) specifies the maximum gain 
   %  of the uncertain dynamics (the default value is MAXGAIN=1).
   %
   %  UH = ULTIDYN(NAME,IOSIZE,'Type','PositiveReal','Bound',BND) specifies 
   %  dynamic uncertainty whose frequency response lies in the half-plane
   %     UH(jw)+UH(jw)' >= 2*BND
   %  The default TYPE is 'GainBounded' (uniform bound on the frequency
   %  response gain).
   %
   %  The "AutoSimplify" property controls how expressions involving 
   %  uncertain blocks are simplified, type "help ultidyn.AutoSimplify"
   %  for details. When sampling the uncertain dynamics with USAMPLE, use
   %  the "SampleStateDimension" property to control how many states each  
   %  sample LTI model has (the default value is 3).
   %
   %  Examples:
   %    % Create a MIMO process model with 20% uncertainty in its frequency 
   %    % response:
   %    Pnom = rss(5,3,4);   % 5 states, 3 outputs, 4 inputs
   %    % Add 20% multiplicative uncertainty to the input of P
   %    P = Pnom*(eye(4) + ultidyn('DeltaP',[4 4],'Bound',0.2))
   %
   %  See also UMARGIN, UREAL, UCOMPLEX, USS, UFRD, CONTROLDESIGNBLOCK.
   
%   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2012 The MathWorks, Inc.
         
   properties (Access = public, Dependent)   
      % Uncertainty quantification [{'GainBounded'} | 'PositiveReal'].
      Type
      % Uncertainty bound.
      Bound
   end
   
   properties (Access = public)   
      % State dimension for random sampling.
      SampleStateDimension = 3;
      % Maximum natural frequency for random sampling. The randomly sampled 
      % uncertain dynamics are no faster than the specified value 
      % (default=Inf).
      SampleMaxFrequency = Inf;
   end
   
   properties (Access = protected)
      % Storage properties (uncoupled)
      % Uncertainty quantification mode (G/P)
      Type_ = 'G';
      % Uncertainty bound
      Bound_
   end
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function boo = isClosed(~)
         boo = false;
      end
      
      function T = toClosed(~)
         T = 'uss';
      end
      % Note: getAttributes never called (first converted to USS)      
   end
    
   %% PUBLIC METHODS
   methods
      
      function blk = ultidyn(name,iosize,varargin)
         ni = nargin;
         if ni==0
            name = 'UNNAMED';  iosize = [1 1];
         elseif ni==1
            iosize = [1 1];
         elseif rem(ni,2)~=0
            error(message('Robust:umodel:ultidyn1'))
         elseif ~(isnumeric(iosize) && isreal(iosize) && ...
               all(iosize>=0 & rem(iosize,1)==0 & isfinite(iosize)))
            error(message('Robust:umodel:ultidyn2'))
         else
            if isscalar(iosize)
               iosize = iosize([1 1]);
            elseif ~isequal(size(iosize),[1 2])
               error(message('Robust:umodel:ultidyn2'))
            end
            iosize = full(double(iosize));
         end
         blk.IOSize_ = iosize;
         blk.Bound_ = 1;
         blk.Ts_ = 0;
         try
            blk.Name = name;
            % Process additional inputs.
            if ~isempty(varargin)
               blk = set(blk,varargin{:});
            end
         catch ME
            throw(ME)
         end
      end
      
      function Value = get.Type(blk)
         % GET method for Type property
         switch blk.Type_
            case 'G'
               Value = 'GainBounded';
            case 'P'
               Value = 'PositiveReal';
         end
      end
      
      function Value = get.Bound(blk)
         % GET method for Bound property
         Value = blk.Bound_;
      end
                           
      function blk = set.Type(blk,Value)
         % SET method for Type property
         T = ltipack.matchKey(Value,{'GainBounded','PositiveReal'});
         if isempty(T)
            error(message('Robust:umodel:ultidyn4'))
         end
         T = T(1);
         if ~strcmp(T,blk.Type_)
            % Switching type
            ios = blk.IOSize_;
            switch T
               case 'P'
                  % Switching to PositiveReal
                  if ios(1)~=ios(2)
                     error(message('Robust:umodel:ultidyn5'))
                  end
                  blk.Bound_ = 0;  
               case 'G'
                  % Switching to GainBounded
                  blk.Bound_ = 1;
            end
            blk.Type_ = T;
         end
      end
      
      function blk = set.Bound(blk,Value)
         % SET method for Bound property
         if ~(isnumeric(Value) && isscalar(Value) && isreal(Value) && isfinite(Value))
            error(message('Robust:umodel:ultidyn6'))
         end
         Value = full(double(Value));
         if strcmp(blk.Type_,'G') && Value<=0
            error(message('Robust:umodel:ultidyn7'))
         end
         blk.Bound_ = Value;
      end
      
      function blk = set.SampleStateDimension(blk,Value)
         % SET method for SampleStateDimension property
         if ~(isnumeric(Value) && isscalar(Value) && isreal(Value) && ...
               isfinite(Value) && Value>0.5)
            error(message('Robust:umodel:ultidyn8'))
         end
         blk.SampleStateDimension = round(full(double(Value)));
      end
      
      function blk = set.SampleMaxFrequency(blk,Value)
         % SET method for SampleMaxFrequency property
         if ~(isnumeric(Value) && isscalar(Value) && isreal(Value) && Value>0)
            error(message('Robust:umodel:ultidyn13'))
         end
         blk.SampleMaxFrequency = full(double(Value));
      end
      
   end
   
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      % INPUTOUTPUTMODEL
      function displaySize(~,sizes)
         % Display for "size(sys)"
         disp(getString(message('Robust:umodel:SizeULTI',sizes(1),sizes(2))))
      end
      
      function blk = uscale_(blk,fact)
         % Scales normalized uncertainty level
         switch blk.Type_
            case 'G'
               blk.Bound_ = fact * blk.Bound_;
            case 'P'
               error(message('Robust:umodel:ultidyn3'))
         end
      end
      
   end
      
   %% HIDDEN INTERFACES
   methods (Hidden)
            
      % CONTROLDESIGNBLOCK
      function Offset = getOffset(blk)
         % Default value is the nominal value
         ios = blk.IOSize_;
         switch blk.Type_
            case 'G'
               Offset = zeros(ios);
            case 'P'
               bnd = blk.Bound_;
               Offset = (bnd + (2*abs(bnd) + 1)) * eye(ios(1));
         end
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         nyu = iosize(blk);
         ioSize = sprintf('%dx%d',nyu(1),nyu(2));
         switch blk.Type_
            case 'G'
               MsgID = 'Robust:umodel:ultidyn11';
            case 'P'
               MsgID = 'Robust:umodel:ultidyn12';
         end
         str = getString(message(MsgID,...
            blk.Name,ioSize,sprintf('%.3g',blk.Bound_),ncopies));
      end
   
   end
   
   %% UTILITIES
   methods (Hidden)
      
      function [R,S,T] = normalizeBlock(blk)
         % Computes R,S,T such that
         %    * LFT(R,blk-S) is normalized
         %    * blk = LFT(T,LFT(R,blk-S))
         % Returns R=[] if blk-S is already normalized (then blk = LFT(T,blk-S))
         S = getOffset(blk);
         [ny,nu] = size(S);
         beta = blk.Bound_;
         switch blk.Type_
            case 'G'
               tau = sqrt(beta);
               T = [S tau*eye(ny);tau*eye(nu) zeros(nu,ny)];
               if abs(beta-1)<100*eps
                  % Already normalized
                  R = [];
               else
                  tau = 1/tau;
                  R = [zeros(ny,nu) tau*eye(ny);tau*eye(nu) zeros(nu,ny)];
               end
            case 'P'
               tau = sqrt(2*(1+2*abs(beta)));  aux = tau*eye(nu);
               T = [S -aux;aux -eye(nu)];
               tau = 1/tau;   aux = tau*eye(nu);
               R = [zeros(nu) -aux;aux -tau*aux];
         end
      end
      
      function Aval = norm2act(blk,Nval)
         % Converts normalized values to actual block values. The transformation is
         %   * A = Bound * N for Type='Gain'
         %   * A = lft(T,N) for Type='PositiveReal'
         if ~isa(Nval,'DynamicSystem')
            error(message('Robust:umodel:norm2act5'))
         end
         sN = size(Nval);
         if ~isequal(sN(1:2),blk.IOSize_)
            error(message('Robust:umodel:norm2act4',blk.IOSize_(1),blk.IOSize_(2)))
         end
         switch blk.Type_
            case 'G'
               Aval = blk.Bound_ * Nval;
            case 'P'
               [~,~,T] = normalizeBlock(blk);
               [Aval,SingularFlag] = lft(T,Nval);
               if SingularFlag
                  warning(message('Robust:umodel:norm2act6'))
               end
         end
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values. The 
         % transformation is 
         %   * N = A/Bound for Type='Gain'
         %   * N = lft(R,A-S) for Type='PositiveReal'
         if ~isa(Aval,'DynamicSystem')
            error(message('Robust:umodel:act2norm5'))
         end
         sA = size(Aval);
         if ~isequal(sA(1:2),blk.IOSize_)
            error(message('Robust:umodel:act2norm4',blk.IOSize_(1),blk.IOSize_(2)))
         end
         switch blk.Type_
            case 'G'
               Nval = (1/blk.Bound_) * Aval;
            case 'P'
               [R,S] = normalizeBlock(blk);
               [Nval,SingularFlag] = lft(R,Aval-S);
               if SingularFlag
                  warning(message('Robust:umodel:act2norm6'))
               end
         end
         if nargout>1
            Ndist = zeros([sA(3:end) 1 1]);
            for ct=1:numel(Ndist)
               Ndist(ct) = getPeakGain(Nval(:,:,ct),1e-6);
            end
         end
      end
      
      function C = randSample_(blk,N)
         % Randomly samples uncertain dynamics. Returns a cell array of
         % ltipack.ssdata objects.
         Ts = blk.Ts_;
         ios = blk.IOSize_;
         Nx = blk.SampleStateDimension;
         XNames = strseq(sprintf('%s_x',getName(blk)),1:Nx);
         % Get max natural frequency
         WMAX = blk.SampleMaxFrequency;
         if ~isfinite(WMAX)
            WMAX = 500*rand;
         end
         if Ts~=0
            % Limit to 1/(2*Ts) in discrete time
            WMAX = min(WMAX,0.5/abs(Ts));
         end
         % Generate N unit-norm continuous-time samples
         for ct=N:-1:1
            D(ct,1) = ltipack.ssdata.rssnb(Nx,ios(1),ios(2),WMAX);
         end
         % Discretize if necessary
         % Note: Tustin transform preserves the Hinf norm
         if Ts~=0
            Options = c2dOptions('Method','tustin');
            for ct=1:N
               D(ct) = utDiscretizeTustin(D(ct),abs(Ts),Options);
            end
         end
         % Scale and transform
         switch blk.Type_
            case 'G'
               alpha = blk.Bound_;
               for ct=1:N
                  D(ct).b = alpha * D(ct).b;
                  D(ct).d = alpha * D(ct).d;
                  D(ct).StateName = XNames;
               end
            case 'P'
               % Note: ny=nu
               [~,~,T] = normalizeBlock(blk);
               nio = ios(1);
               DT = createGain(D(1),T);
               for ct=1:N
                  D(ct) = lft(DT,D(ct),nio+1:2*nio,nio+1:2*nio,1:nio,1:nio);
                  D(ct).StateName = XNames;
               end
         end
         C = num2cell(D);
      end

      function blkN = getNormalizedForm(blk)
         % Returns normalized version of block (see LFTDATA)         
         blkN = ultidyn(sprintf('%sNormalized',getName(blk)),iosize(blk));
         blkN.Ts_ = blk.Ts_;
      end
      
      function WCP = getWorstCasePerturbation(blk,nprt,Freq)
         % Constructs dynamic perturbation
         % Called by rsEngine and a local function in lftdataFRD/WCGARRAYMAX.
         % The normalized perturbation NPRT is either a 1-by-2 cell containing 
         % a column vector and a row vector, or a 2D double array.
         if Freq<0
            % Map negative frequency to positive frequency using that
            % dynamic perturbation from mkPert is always real 
            Freq = -Freq;  nprt = conj(nprt);
         end
         Ts = blk.Ts_;
         if iscell(nprt)
            wlvec = nprt{1};
            wrvec = nprt{2};
         else
            [u,s,v] = svd(nprt);
            smax = sqrt(s(1,1));
            wlvec = u(:,1)*smax;
            wrvec = smax*v(:,1)';
         end
         if all(wlvec==0)
            rs = numel(wlvec);  cs = numel(wrvec);
            Ap = []; Bp = zeros(0,cs); Cp = zeros(rs,0); Dp = zeros(rs,cs);
         else
            [Ap,Bp,Cp,Dp] = rctutil.mkPert(wlvec,wrvec,Freq,Ts);
         end
         WCP = ltipack.ssdata(Ap,Bp,Cp,Dp,[],Ts);
      end
      
   end
       
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
   end
   
end
