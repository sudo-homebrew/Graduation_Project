classdef (InferiorClasses = {? matlab.graphics.axis.Axes, ? matlab.ui.control.UIAxes}) umargin < UncertainDynamics
   %UMARGIN  Model gain and phase uncertainty in feedback loops.
   %
   %   This block models gain and phase variations in individual feedback
   %   channels as a dynamic multiplicative factor F(s) taking values in a
   %   disk centered on the real axis. This disk contains the nominal value
   %   F=1 and is characterized by its intersection [GMIN,GMAX] with the real
   %   axis. GMIN<1 and GMAX>1 are lower and upper bounds on the relative gain
   %   change, for example, [0.8,1.5] means that the gain can vary between 80%
   %   and 150% of its nominal value. The amount of phase uncertainty is
   %   determined by the disk geometry.
   %
   %   Since robust stability for such gain/phase uncertainty is equivalent to
   %   the disk-based gain margin DGM=[GMIN,GMAX] and the corresponding phase
   %   margin, you can use the UMARGIN block to verify disk margins when
   %   analyzing robustness (see ROBSTAB) or to enforce suitable disk margins
   %   when designing or tuning controllers (see MUSYN and SYSTUNE).
   %
   %   F = UMARGIN(NAME,DGM) models relative gain uncertainty in the range
   %   DGM=[GMIN,GMAX] with GMIN<1 and GMAX>1. This range is for phase held
   %   at its nominal value. DGM can be seen as the desired disk-based
   %   gain margin (see DISKMARGIN). Use GETDGM to compute a suitable DGM when
   %   you have both gain and phase uncertainty.
   %
   %   For scalar GM>1, UMARGIN(NAME,GM) is the same as UMARGIN(NAME,[1/GM,GM]) 
   %   and specifies that the gain can increase or decrease by a factor GM
   %   (the desired gain margin). This corresponds to the "balanced" case 
   %   SIGMA=0 for DISKMARGIN.
   %
   %   The "GainChange" and "PhaseChange" properties summarize the gain and
   %   phase uncertainty. The "AutoSimplify" property controls how expressions
   %   involving uncertain blocks are simplified. When sampling F(s) with
   %   USAMPLE, use the "SampleStateDimension" property to control how many
   %   states each sample has (the default value is 3).
   %
   %   Example:
   %     % Open-loop response with pole uncertainty
   %     a = ureal('a',10,'Range',[9 12])
   %     L = tf(25,[1 a a a]);
   %     % Specify gain variations between 70% and 150% of the nominal
   %     % gain and phase variation of plus or minus 30 degrees
   %     DGM = getDGM([0.7,1.5],30,'tight');
   %     F = umargin('F',DGM);
   %     % Close the loop and analyze robustness
   %     CL = feedback(L*F,1);
   %     SM = robstab(CL)
   %     % This feedback loop can only tolerate 81% of specified uncertainty.
   %     % This corresponds to gain variations between 57% and 141% and
   %     % phase variation of plus or minus 24 degrees:
   %     uscale(F,SM.LowerBound)
   %
   %   See also UMARGIN/PLOT, GETDGM, DISKMARGIN, ULTIDYN, UREAL,
   %   UCOMPLEX, USS, UFRD, CONTROLDESIGNBLOCK.
   
   %   Copyright 2020 The MathWorks, Inc.
   
   properties (Access = public, Dependent)
      % Bounds on relative gain change (in absolute units).
      %
      % This is of the form [GMIN,GMAX] with GMIN<1 and GMAX>1. For
      % example, [0.8 1.5] means that the gain can vary between 80% and
      % 150% of its nominal value. Note that GMIN can go negative for 
      % SIGMA<0, which models possible sign changes in the loop gain.
      GainChange
   end
   
   properties (GetAccess = public, SetAccess = private, Dependent)
      % Bounds on absolute phase change (in degrees).
      %
      % This is of the form [PMIN,PMAX] with PMIN<0 and PMAX>0. For
      % example, [-30 30] means that the phase can change by 30 degrees
      % up or down.
      PhaseChange
   end
   
   properties (Access = public, Dependent)
      % Normalized uncertainty level (positive scalar).
      % 
      % This is the parameter ALPHA in the normalized uncertainty model,
      % see GM2DM for details.
      DiskMargin
      % Skew (real scalar)
      % 
      % This is the parameter SIGMA in the normalized uncertainty model, 
      % see GM2DM for details.
      Skew
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
      % Disk-based gain margin
      DGM_ = [0.5,2];
   end
   
   properties (Dependent, Hidden)
      Eccentricity  % 2020a alias for Skew
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
      
      function blk = umargin(name,varargin)
         blk.IOSize_ = [1 1];
         blk.Ts_ = 0;
         if nargin==0
            blk.Name = 'UNNAMED';
            return
         end
         % Dissect input list
         ni = numel(varargin);
         DataInputs = 0;
         PVStart = ni+1;
         for ct=1:ni
            if ischar(varargin{ct}) || isstring(varargin{ct})
               PVStart = ct;   break
            else
               DataInputs = DataInputs+1;
            end
         end
         % Validate DGM
         switch DataInputs
            case 0
               DGM = blk.DGM_;
            case 1
               DGM = umargin.checkDGM(varargin{1});
               if isempty(DGM)
                  error(message('Robust:umodel:umargin2'))
               end
            otherwise
               error(message('Robust:umodel:umargin3'))
         end
            
         try
            blk.Name = name;
            blk.DGM_ = DGM;
            % Process additional inputs.
            if PVStart<=ni
               blk = set(blk,varargin{PVStart:ni});
            end
         catch ME
            throw(ME)
         end
      end
            
      function DGM = get.GainChange(blk)
         DGM = blk.DGM_;
      end
      
      function DPM = get.PhaseChange(blk)
         DPM = getDPM(blk.DGM_);
      end
      
      function Value = get.DiskMargin(blk)
         Value = gm2dm(blk.DGM_);
      end
      
      function Value = get.Skew(blk)
         [~,Value] = gm2dm(blk.DGM_);
      end
      function Value = get.Eccentricity(blk)
         [~,Value] = gm2dm(blk.DGM_);
      end
      
      function blk = set.GainChange(blk,DGM)
         % SET method for GainChange property
         DGM = umargin.checkDGM(DGM);
         if isempty(DGM)
            error(message('Robust:umodel:umargin7'))
         end
         blk.DGM_ = DGM;
      end
      
      function blk = set.DiskMargin(blk,alpha)
         % SET method for DiskMargin property
         if ~(isnumeric(alpha) && isscalar(alpha) && isreal(alpha) && alpha>0 && alpha<Inf)
            error(message('Robust:umodel:umargin1'))
         end
         alpha = full(double(alpha));
         sigma = blk.Skew;
         % Enforce alpha*|1+sigma|<2 which is needed for stability of F(s) and
         % equivalent to GMIN<1<GMAX
         if alpha * abs(1+sigma)>=2
            error(message('Robust:umodel:umargin4'))
         end
         blk.DGM_ = dm2gm(alpha,sigma);
      end
      
      function blk = set.Skew(blk,sigma)
         % SET method for Skew property
         if ~(isnumeric(sigma) && isscalar(sigma) && isreal(sigma) && isfinite(sigma))
            error(message('Control:utility:dm2gm2'))
         end
         alpha = blk.DiskMargin;
         sigma = full(double(sigma));
         % Enforce alpha*|1+sigma|<2 which is needed for stability of F(s) and
         % equivalent to GMIN<1<GMAX
         if alpha * abs(1+sigma)>=2
            error(message('Robust:umodel:umargin4'))
         end
         blk.DGM_ = dm2gm(alpha,sigma);
      end
      function blk = set.Eccentricity(blk,sigma)
         blk.Skew = sigma;
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
      function displaySize(~,~)
         % Display for "size(sys)"
         disp(getString(message('Robust:umodel:SizeUMARGIN')))
      end
      
      function blk = uscale_(blk,fact)
         % Scales normalized uncertainty level
         [alpha0,sigma] = gm2dm(blk.DGM_);
         alpha = alpha0*fact;
         if alpha*abs(1+sigma)>2*(1-10*eps)
            error(message('Robust:umodel:umargin8',blk.Name,...
               sprintf('%.4g',2/abs(1+sigma)/alpha0)))
         end
         blk.DGM_ = dm2gm(alpha,sigma);
      end
      
   end
   
   %% HIDDEN INTERFACES
   methods (Hidden)
      
      % CONTROLDESIGNBLOCK
      function Offset = getOffset(~)
         % Default value is the nominal value
         Offset = 1;
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         str = getString(message('Robust:umodel:umargin5',...
            blk.Name,sprintf('[%.3g,%.3g]',blk.DGM_),...
            sprintf('%.3g',blk.PhaseChange(2)),ncopies));
      end
      
   end
   
   %% UTILITIES
   methods (Hidden)
      
      function [R,S,T] = normalizeBlock(blk)
         % Computes R,S,T such that
         %    * LFT(R,blk-S) is normalized
         %    * blk = LFT(T,LFT(R,blk-S))
         % Returns R=[] if blk-S is already normalized (then blk = LFT(T,blk-S))
         [alpha,sigma] = gm2dm(blk.DGM_);
         aux = sqrt(alpha);
         S = 1;
         T = [1 aux;aux alpha*(1+sigma)/2];
         if abs(1+sigma)+abs(1-alpha)<100*eps
            % Already normalized
            R = [];
         else
            % delta = feedback(blk-S,(1+sigma)/2)/alpha
            R = [0 1/aux;1/aux -(1+sigma)/2];
         end
      end
      
      function Aval = norm2act(blk,Nval)
         % Converts normalized values to actual block values.
         if ~isa(Nval,'DynamicSystem')
            error(message('Robust:umodel:norm2act5'))
         elseif ~issiso(Nval)
            error(message('Robust:umodel:norm2act7'))
         end
         [~,~,T] = normalizeBlock(blk);
         [Aval,SingularFlag] = lft(T,Nval);
         if SingularFlag
            warning(message('Robust:umodel:norm2act6'))
         end
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values.
         if ~isa(Aval,'DynamicSystem')
            error(message('Robust:umodel:act2norm5'))
         elseif ~issiso(Aval)
            error(message('Robust:umodel:act2norm7'))
         end
         sA = size(Aval);
         [R,S] = normalizeBlock(blk);
         [Nval,SingularFlag] = lft(R,Aval-S);
         if SingularFlag
            warning(message('Robust:umodel:act2norm6'))
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
            D(ct,1) = ltipack.ssdata.rssnb(Nx,1,1,WMAX);
         end
         % Discretize if necessary
         % Note: Tustin transform preserves the Hinf norm
         if Ts~=0
            Options = c2dOptions('Method','tustin');
            for ct=1:N
               D(ct) = utDiscretizeTustin(D(ct),abs(Ts),Options);
            end
         end
         % Transform back to actual values
         [~,~,T] = normalizeBlock(blk);
         DT = createGain(D(1),T);
         for ct=1:N
            D(ct) = lft(DT,D(ct),2,2,1,1);
            D(ct).StateName = XNames;
         end
         C = num2cell(D);
      end

      function blkN = getNormalizedForm(blk)
         % Returns normalized version of block (see LFTDATA)         
         blkN = ultidyn(sprintf('%sNormalized',getName(blk)),[1 1],'Ts',blk.Ts_);
      end
      
      function WCP = getWorstCasePerturbation(blk,nprt,Freq)
         % Constructs dynamic perturbation
         % The normalized perturbation NPRT is either a 1-by-2 cell containing 
         % a column vector and a row vector, or a 2D double array. Returns
         % normalized dynamic perturbation DELTA(s).
         if Freq<0
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
      
      function blk = loadobj(s)
         if isa(s,'umargin')
            blk = s;
            blk.Version_ = ltipack.ver();
         end
      end
      
      function DGM = checkDGM(DGM)
         % Validates user input for DGM. Returns [] if invalid
         if ~(isnumeric(DGM) && isreal(DGM))
            DGM = []; return
         end
         DGM = full(double(DGM));
         if isscalar(DGM)
            DGM = [1./DGM , DGM];
         elseif ~(isrow(DGM) && numel(DGM)==2)
            DGM = []; return
         end
         % Note: Impose GMIN<1 and GMAX>1 to prevent sigma=Inf
         if ~(DGM(1)<1 && DGM(2)>1 && DGM(1)>-Inf && DGM(2)<Inf)
            DGM = []; return
         end
      end
      
   end
   
end
