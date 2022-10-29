function varargout = diskmargin(varargin)
%DISKMARGIN  Disk-based stability margins of feedback loops.
%
%   DISKMARGIN computes disk-based stability margins for SISO or MIMO
%   feedback loops. See documentation for details on disk margin concept.
%
%   [DM,MM] = DISKMARGIN(L) analyzes the N-channel negative-feedback loop
%
%         u --->O---->[ L ]----+---> y
%             - |              |
%               +<-------------+
%
%   The N-by-N loop transfer matrix L can be any LTI model. DISKMARGIN
%   models gain/phase variations in the feedback channel L(j,j) as the
%   multiplicative uncertainty
%      L(j,j) * (1+DELTA/2) / (1-DELTA/2),   |DELTA| < ALPHA.
%   DISKMARGIN estimates the largest perturbation size ALPHA for which the
%   feedback loop remains stable. The structure DM(j) contains the disk
%   margin for the feedback channel L(j,j) with all other loops closed.
%   The structure MM contains the multi-loop disk margin, the largest ALPHA
%   for which stability is maintained when subjecting all feedback channels
%   to independent and concurrent gain/phase variations |DELTAj| < ALPHA.
%
%   MMIO = DISKMARGIN(P,C) estimates the stability margins of the
%   negative-feedback loop
%
%         u --->O--->[ C ]-->[ P ]---+---> y
%             - |                    |
%               +<-------------------+
%
%   when subjecting all input and output channels of the plant P to
%   independent and concurrent gain/phase variations.
%
%   The structures DM, MM, and MMIO have the following fields:
%     LowerBound   Lower bound on disk margin ALPHA
%     UpperBound   Upper bound on disk margin ALPHA
%     Frequency    Frequency F with weakest disk margin (in rad/TimeUnit)
%     DiskMargin   Guaranteed disk margin (same as LowerBound)
%     GainMargin   Sustainable gain-only variation (absolute value)
%     PhaseMargin  Sustainable phase-only variation (degrees)
%     WorstPerturbation  Smallest destabilizing gain/phase variation WGP.
%                        WGP has size ALPHA in DELTA coordinates and
%                        FEEDBACK(L*WGP,EYE(N)) has a pole at s=j*F.
%
%   [...] = DISKMARGIN(...,SIGMA) specifies an additional "skew" parameter
%   SIGMA. This changes the uncertainty model to
%      L(j,j) * (1+DELTA*(1-SIGMA)/2)/(1-DELTA*(1+SIGMA)/2),  |DELTA|<ALPHA
%   and uses the shifted sensitivity function
%      S + (SIGMA-1)*I/2,   S = inv(I+L)
%   to calculate the margins. Special cases include
%      SIGMA=0    "Balanced" margins based on (S-T)/2 (default)
%      SIGMA=1    Margins based on sensitivity S
%      SIGMA=-1   Margins based on complementary sensitivity T=I-S.
%   Use SIGMA=0 when the gain can increase or decrease by the same factor,
%   SIGMA<0 when it can decrease by more than it can increase, and SIGMA>0
%   when it can increase by more than it can decrease.
%
%   Note: Use diskmarginplot(DM.GainMargin,...) to visualize the stable
%   range of gain/phase variations and interpret the disk-based margins.
%
%   Example: Analyze stability of two-input, two-output feedback loop.
%      a = [-0.2 10;-10 -0.2]; b = eye(2); c = [1 8;-10 1];
%      P = ss(a,b,c,0);
%      K = [1 -2;0 1];
%      [DMI,MMI] = diskmargin(K*P); % disk margins at plant inputs
%      [DMO,MMO] = diskmargin(P*K); % disk margins at plant outputs
%      MMIO = diskmargin(P,K); % vary gain/phase at both inputs and outputs
%      diskmarginplot(K*P)     % plot disk margins at plant inputs
%      % view gain/phase variations for which stability is guaranteed
%      diskmarginplot(MMI.DiskMargin)
%
%   See also DISKMARGINPLOT, MARGIN, ALLMARGIN, WCDISKMARGIN, UMARGIN.

%   Author(s): MUSYN and MathWorks
%   Copyright 2018-2020 The MathWorks, Inc.
%   The authors thank Christopher Mayhew for helpful suggestions.
narginchk(1,3)
ni = nargin;
no = nargout;
if ni==3 || (isnumeric(varargin{ni}) && isscalar(varargin{ni}))
   sigma = varargin{ni}; ni = ni-1;
   if ~(isnumeric(sigma) && isscalar(sigma) && isreal(sigma) && isfinite(sigma))
      error(message('Robust:analysis:InvalidE'))
   end
else
   sigma = 0;  % balanced
end

% Construct L
if ni==1
   nargoutchk(0,2)
   L = varargin{1};
   [nL,nu] = size(L);
   if nL~=nu
      error(message('Robust:analysis:NonSquareLoopGain'))
   end
   ComputeSISO = true;
   ComputeMIMO = (no>1 && nL>1);
else
   nargoutchk(0,1)
   P = varargin{1};  C = varargin{2};
   [nyP,nuP,~] = size(P); % watch for P or C = double array
   [nyC,nuC,~] = size(C);
   if nuP~=nyC
      error(message('Robust:analysis:OutputCInputPMismatch'))
   elseif nyP~=nuC
      error(message('Robust:analysis:InputCOutputPMismatch'))
   end
   try
      L = [ss(zeros(nyP)) P;-C ss(zeros(nyC))]; % watch for P or C = double array
   catch ME
      throw(ME)
   end
   nL = nyP+nyC;
   ComputeSISO = false;
   ComputeMIMO = true;
end

% Convert L to @ss or @frd
if nL==0
   error(message('Robust:analysis:EmptyL'))
end
try
   if isa(L,'FRDModel')
      L = frd(L);
   else
      L = ss(L);
   end
catch ME
   throw(ME)
end
if ~isfinite(L)
   error(message('Robust:analysis:diskmargin1'))
end
TU = L.TimeUnit;

% Compute margins
if ComputeSISO
   DM = diskmargin_(L,sigma,'siso');
   DM = localMarginData(DM,sigma,TU);
end
if ComputeMIMO
   MM = diskmargin_(L,sigma,'mimo');
   MM = localMarginData(MM,sigma,TU);
end

% Assign outputs
if ni==1
   varargout{1} = DM;
else
   % Split worst perturbation into input and output perturbations
   for j=1:numel(MM)
      WP = MM(j).WorstPerturbation;
      WPO = subparen(WP,{1:nyP, 1:nyP});
      WPI = subparen(WP,{nyP+1:nyP+nuP, nyP+1:nyP+nuP});
      MM(j).WorstPerturbation = struct('Input',WPI,'Output',WPO);
   end
   varargout{1} = MM;
end
if no>1
   if nL>1
      varargout{2} = MM;
   else
      % SISO loop: MM = DM
      varargout{2} = DM;
   end
end


function S = localMarginData(S,sigma,TU)
% Add fields DiskMargin, GainMargin, and PhaseMargin
FN = {'GainMargin';'PhaseMargin';'DiskMargin';...
   'LowerBound';'UpperBound';'Frequency'; 'WorstPerturbation'};
if isempty(S)
   S = cell2struct(cell([numel(FN) size(S)]),FN,1);
else
   for ct=1:numel(S)
      DM = S(ct).LowerBound;
      S(ct).DiskMargin = DM;
      [S(ct).GainMargin,S(ct).PhaseMargin] = dm2gm(DM,sigma);
      WP = ss.make(S(ct).WorstPerturbation);
      WP.TimeUnit = TU;
      S(ct).WorstPerturbation = WP;
   end
   S = orderfields(S,FN);
end