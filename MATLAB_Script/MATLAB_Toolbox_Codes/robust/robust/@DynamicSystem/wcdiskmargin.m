function [WCDM,WCU,Info] = wcdiskmargin(varargin)
%WCDISKMARGIN  Worst-case stability margins of uncertain feedback loops.
%
%   WCDISKMARGIN is the counterpart of DISKMARGIN for uncertain feedback
%   loops. It calculates guaranteed minimum stability margins and
%   identifies the "worst-case" perturbation yielding the weakest margins.
%   See DISKMARGIN for more details on disk margins.
%
%   [WCDM,WCU] = WCDISKMARGIN(L,'siso') estimates the worst-case
%   loop-at-a-time disk margins for the N-channel negative-feedback loop:
%
%         u --->O---->[ L ]----+---> y
%             - |              |
%               +<-------------+
%
%   For each SISO loop L(j,j), the worst-case disk margins provide minimum
%   guaranteed gain and phase margins (in the classical sense) and help
%   determine how much gain/phase variation the feedback loop can sustain.
%   The loop transfer L can be a USS or UFRD model. The N-by-1 struct
%   arrays WCDM and WCU contain the worst-case margins and worst-case
%   perturbations for each SISO loop. WCU contains (actual) values for each
%   uncertain element of L.
%
%   [WCMM,WCU] = WCDISKMARGIN(L,'mimo') estimates the worst-case
%   multi-loop disk margins for the same feedback loop. This further
%   guarantees stability when subjecting all feedback channels to
%   independent and concurrent gain/phase variations.
%
%   [WCMMIO,WCU] = WCDISKMARGIN(P,C) computes the worst-case margins of
%   the negative-feedback loop
%
%         u --->O--->[ C ]-->[ P ]---+---> y
%             - |                    |
%               +<-------------------+
%
%   when subjecting all input and output channels of the plant P to
%   independent and concurrent gain/phase variations.
%
%   The structures WCDM, WCMM, and WCMMIO have the following fields:
%          LowerBound   Lower bound on worst-case disk margin
%          UpperBound   Upper bound on worst-case disk margin
%   CriticalFrequency   Frequency F with weakest margins (in rad/TimeUnit)
%          DiskMargin   Minimum guaranteed disk margin (same as LowerBound)
%          GainMargin   Minimum guaranteed gain margin (absolute value)
%         PhaseMargin   Minimum guaranteed phase margin (degrees)
%   WorstPerturbation   Smallest destabilizing gain/phase variation WGP for
%                       the worst combination WCU of uncertain elements.
%                       FEEDBACK(USUBS(L,WCU)*WGP,EYE(N)) has a pole at j*F.
%
%   [...] = WCDISKMARGIN(...,SIGMA) specifies an additional "skew" parameter
%   SIGMA, see DISKMARGIN for details.
%
%   [...] = WCDISKMARGIN(...,OPTIONS) specifies additional options, see
%   wcOptions for details.
%
%   [WC*,WCU,INFO] = WCDISKMARGIN(L,...) returns a structure INFO with
%   additional analysis data:
%      Model   When L is an array of models, index of model with smallest
%              disk margin
%      Frequency  M-by-1 vector of frequency points
%      Bounds  M-by-2 array of lower and upper bounds for the worst-case
%              disk margin at each frequency point
%      WorstPerturbation  M-by-1 structure of worst-case uncertain element
%              values at each frequency point
%      Sensitivity  Disk margin sensitivity (in percent) to each uncertain
%              element (when the "Sensitivity" option is set to 'on').
%
%   See also WCOPTIONS, WCDISKMARGINPLOT, DISKMARGIN, WCGAIN.

%   Copyright 1986-2020 The MathWorks, Inc.

% Input Check
narginchk(2,4)
ni = nargin;
no = nargout;

% Construct L
if ischar(varargin{2}) || isstring(varargin{2})
   % Second argument is JOB
   L = varargin{1};
   [nL,nu] = size(L);
   if nL~=nu
      error(message('Robust:analysis:NonSquareLoopGain'))
   end
   JOB = ltipack.matchKey(varargin{2},{'siso','mimo'});
   if isempty(JOB)
      error(message('Robust:analysis:WCDISKMARGIN1'))
   end
   IOFLAG = false;
elseif isnumeric(varargin{2}) || isa(varargin{2},'InputOutputModel')
   P = varargin{1};
   C = varargin{2};
   JOB = 'mimo';
   [nyC,nuC,~] = size(C);  % watch for P or C = double array
   [nyP,nuP,~] = size(P);
   if nuP~=nyC
      error(message('Robust:analysis:OutputCInputPMismatch'))
   elseif nyP~=nuC
      error(message('Robust:analysis:InputCOutputPMismatch'))
   end
   try
      L = [ss(zeros(nyP)) P;-C ss(zeros(nyC))];  % watch for P or C = double array
   catch ME
      throw(ME)
   end
   nL = nyP+nyC;
   IOFLAG = true;
else
   error(message('Robust:analysis:WCDISKMARGIN5'))
end
if nL==0
   error(message('Robust:analysis:EmptyL'))
elseif nmodels(L)==0
   error(message('Robust:analysis:WCDISKMARGIN3'))
end

% Get options and skew
sigma = 0;  % balanced
opt = wcOptions();
if ni>2
   if ni==4
      sigma = varargin{3};  opt = varargin{4};
   elseif isnumeric(varargin{3})
      sigma = varargin{3};
   else
      opt = varargin{3};
   end
   if ~(isnumeric(sigma) && isscalar(sigma) && isreal(sigma) && isfinite(sigma))
      error(message('Robust:analysis:InvalidE'))
   end
   if ~isa(opt,'rctoptions.wcAnalysis')
      error(message('Robust:analysis:WCDISKMARGIN4'))
   end
end
DisplayFlag = strcmp(opt.Display,'on');

try
   % Convert L to USS or UFRD, validate options
   [L,opt] = robSetUp(L,opt);
   
   % Adjust options to eliminate wasteful computation
   if no<3
      opt.VaryFrequency = 'off';
      if ~DisplayFlag
         opt.Sensitivity = 'off';
      end
   end
   
   % Check ULEVEL compatibility with UREAL ranges
   [NL,NR] = rctutil.boundIP(struct2cell(L.Uncertainty));
   MaxULevel = min([NR;abs(NL)]);
   if opt.ULevel>MaxULevel
      error(message('Robust:analysis:WCGAIN3',num2str(MaxULevel,'%.2g')));
   end
   
   % Compute disk margins
   % SM,Info,IRD are nL-by-1 struct arrays when JOB = 'siso'
   [SM,Info,IRD] = wcdiskmargin_(L,sigma,JOB,opt);
catch ME
   throw(ME)
end

% Construct WCDM
TU = L.TimeUnit;
WCDM = localMarginData(SM,sigma,TU);
if IOFLAG
   % Split gain/phase perturbation
   for j=1:numel(WCDM)
      WP = WCDM(j).WorstPerturbation;
      WPO = subparen(WP,{1:nyP, 1:nyP});
      WPI = subparen(WP,{nyP+1:nyP+nuP, nyP+1:nyP+nuP});
      WCDM(j).WorstPerturbation = struct('Input',WPI,'Output',WPO);
   end
end


% Show report
if DisplayFlag
   localShowReport(WCDM,Info,IRD,opt,TU,sigma)
end

% Construct WCU
if no>1
   % Convert low-level ltipack.ssdata into @ss
   try
      WCU = arrayfun(@(S) ss.makeS(S.WCU,TU),IRD);
   catch
      % WCU does not have consistent fields
      error(message('Robust:analysis:WCDISKMARGIN2'))
   end
end

% Construct INFO
if no>2
   for j=1:numel(Info)
      Info(j).WorstPerturbation = ss.makeS(Info(j).WorstPerturbation,TU);
   end
end


%--------------- Local functions ----------------------------
function S = localMarginData(S,sigma,TU)
% Add fields DiskMargin, GainMargin, and PhaseMargin
for ct=1:numel(S)
   DM = S(ct).LowerBound;
   S(ct).DiskMargin = DM;
   [S(ct).GainMargin,S(ct).PhaseMargin] = dm2gm(DM,sigma);
   WP = ss.make(S(ct).WorstPerturbation);
   WP.TimeUnit = TU;
   S(ct).WorstPerturbation = WP;
end
S = orderfields(S,{'GainMargin';'PhaseMargin';'DiskMargin';...
   'LowerBound';'UpperBound';'CriticalFrequency';'WorstPerturbation'});

function localShowReport(WCDM,Info,IRD,Opt,TimeUnit,sigma)
nL = numel(WCDM);
funits = ['rad/' TimeUnit];
ulevel = round(100*Opt.ULevel);
if IRD(1).StableFlag
   % Nominally stable
   indent = '';
   for ct=1:nL
      if nL>1
         fprintf('\n%s\n',getString(message('Robust:analysis:WCDMChannel',int2str(ct))))
         indent = '  ';
      end
      if WCDM(ct).UpperBound==0
         disp([indent getString(message('Robust:analysis:WCDMZero1'))])
         m1 = getString(message('Robust:analysis:WCGAINInf3'));
         disp([indent ' -- ' m1])
         m2 = getString(message('Robust:analysis:WCGAINInf5'));
         disp([indent ' -- ' m2])
      elseif WCDM(ct).LowerBound==0
         freq = WCDM(ct).CriticalFrequency;
         [GM,PM] = dm2gm(WCDM(ct).UpperBound,sigma);
         disp([indent getString(message('Robust:analysis:WCDMZero2',...
            num2str(WCDM(ct).UpperBound,'%0.3g'),num2str(freq,'%0.3g'),funits))])
         disp([indent getString(message('Robust:analysis:WCDMZero3',...
            num2str(mag2db(GM(2)),'%0.3g')))])
         disp([indent getString(message('Robust:analysis:WCDMZero4',...
            num2str(PM(2),'%0.3g')))])
         m1 = getString(message('Robust:analysis:WCGAINInf4'));
         disp([indent ' -- ' m1])
         m2 = getString(message('Robust:analysis:WCGAINInf5'));
         disp([indent ' -- ' m2])
      else
         freq = WCDM(ct).CriticalFrequency;
         [GM,PM] = dm2gm(WCDM(ct).UpperBound,sigma);
         disp([indent getString(message('Robust:analysis:WCDMFinite1',...
            num2str(WCDM(ct).LowerBound,'%0.3g')))])
         disp([indent getString(message('Robust:analysis:WCDMFinite2',...
            num2str(mag2db(WCDM(ct).GainMargin(2)),'%0.3g')))])
         disp([indent getString(message('Robust:analysis:WCDMFinite3',...
            num2str(WCDM(ct).PhaseMargin(2),'%0.3g')))])
         m1 = getString(message('Robust:analysis:WCGAINFinite2',...
            num2str(ulevel,'%d')));
         disp([indent ' -- ' m1])
         m2 = getString(message('Robust:analysis:WCDMFinite4',...
            num2str(mag2db(GM(2)),'%0.3g'),num2str(freq,'%0.3g'),funits));
         disp([indent ' -- ' m2])
         m3 = getString(message('Robust:analysis:WCDMFinite5',...
            num2str(PM(2),'%0.3g'),num2str(freq,'%0.3g'),funits));
         disp([indent ' -- ' m3])
         if strcmp(Opt.Sensitivity,'on')
            str = rctutil.makeSensMessage(Info(ct).Sensitivity,4);
            nr = size(str,1);
            disp([repmat(indent,nr,1),str])
         end
      end
   end
else
   % Nominally unstable
   if IRD(1).hasID
      disp(getString(message('Robust:analysis:WCDMUnstableNomPade',int2str(Opt.PadeN))))
   else
      disp(getString(message('Robust:analysis:WCDMUnstableNom')))
   end
end
