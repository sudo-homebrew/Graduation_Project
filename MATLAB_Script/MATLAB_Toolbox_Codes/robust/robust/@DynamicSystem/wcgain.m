function [WCG,WCU,Info] = wcgain(sys,varargin)
%WCGAIN   Worst-case gain of uncertain system.
%
%   [WCG,WCU] = WCGAIN(USYS) calculates the worst-case peak gain of the  
%   uncertain system USYS (USS or UFRD model). Here "peak gain" refers to
%   the maximum gain over frequency (H-infinity norm), and "worst-case"
%   refers to the maximum value over the uncertainty range modeled in USYS.
%   WCGAIN returns the following data:
%     WCG  Structure with fields:
%          LowerBound: Lower bound on worst-case gain
%          UpperBound: Upper bound on worst-case gain
%          CriticalFrequency: Frequency (in radians/TimeUnit) where
%                             the gain peaks.
%     WCU  Worst-case perturbation. WCU is a structure containing (actual) 
%          values for each uncertain element of USYS. Use USUBS(USYS,WCU)
%          to apply this perturbation to the nominal system.
%
%   [...] = WCGAIN(USYS,{WMIN,WMAX}) computes the worst-case gain in the
%   frequency interval [WMIN,WMAX].
%
%   [...] = WCGAIN(USYS,W) specifies a vector W of frequency points
%   (in radians/TimeUnit) where to compute the gain.
%
%   [...] = WCGAIN(USYS,...,OPTIONS) specifies additional options, see
%   wcOptions for details.
%
%   [WCG,WCU,INFO] = WCGAIN(USYS,...) returns a structure INFO with the
%   following fields:
%      Model   When USYS is an array of models, index of model with largest
%              peak gain
%      Frequency  N-by-1 vector of frequency points
%      Bounds  N-by-2 array of lower and upper bounds for the worst-case
%              gain at each frequency point
%      WorstPerturbation  N-by-1 structure of worst-case perturbations
%              at each frequency point
%      Sensitivity  WCG sensitivity (in percent) to each uncertain element
%              (when the "Sensitivity" option is set to 'on').
%
%   Example: Consider a second-order plant with 20% relative uncertainty:
%      delta = ultidyn('delta',[1 1]); 
%      G = tf(1,[1 .6 1]) * (1+0.2*delta);
%   Close the loop on a PID controller and compute the worst-case
%   sensitivity:
%      C = pid(2.3,1,1.3,0.01);
%      S = feedback(1,G*C);   % sensitivity function
%      [WCG,WCU] = wcgain(S);
%   The worst-case gain of S is about 1.23 at the frequency 2.37 rad/s. Set
%   the uncertain elements to the values in WCU to confirm this result
%      WCS = usubs(S,WCU);
%      getPeakGain(WCS,1e-6)
%   Verify that the worst-case delta is a unit-gain perturbation:
%      sigma(WCU.delta)
%   
%   Use WCSIGMA to visualize the nominal and worst-case gain of SYS as a 
%   function of frequency.
%
%   See also WCOPTIONS, WCSIGMAPLOT, WCDISKMARGIN, getPeakGain, SIGMA, MUSSV, USS, UFRD.

%   Author(s): MUSYN, P. Gahinet
%   Copyright 2003-2016 The MathWorks, Inc.
narginchk(1,3)
nin = nargin;
nout = nargout;
% Get options
MaxOverFreqOff = false;
if nin>1 && ~(isnumeric(varargin{end}) || iscell(varargin{end}))
   opt = varargin{end};  nin = nin-1;
   if isa(opt,'rctoptions.wcgain')
      % Remap
      if strcmp(opt.MaxOverArray,'off') && nmodels(sys)>1
         % Support iterator syntax before 16b
         [WCG,WCU,Info] = localVectorizedCall(sys,varargin{:});
         return
      end
      MaxOverFreqOff = strcmp(opt.MaxOverFrequency,'off');
      opt = wcOptions('Sensitivity',opt.Sensitivity,...
         'SensitivityPercent',opt.VaryUncertainty);
   elseif ~isa(opt,'rctoptions.wcAnalysis')
      error(message('Robust:analysis:WCGAIN1'))
   end
else
   opt = wcOptions();
end
DisplayFlag = strcmp(opt.Display,'on');

try
   % Convert SYS to USS or UFRD, validate options
   [sys,opt] = robSetUp(sys,opt,varargin{1:nin-1});

   % Adjust options to eliminate wasteful computation
   if MaxOverFreqOff
      opt.VaryFrequency = 'on';  % or backward compatibility
   elseif nout<3
      opt.VaryFrequency = 'off';
   end
   if ~DisplayFlag && nout<3
      opt.Sensitivity = 'off';
   end
   
   % Check ULEVEL compatibility with UREAL ranges
   [NL,NR] = rctutil.boundIP(struct2cell(sys.Uncertainty));
   MaxULevel = min([NR;abs(NL)]);
   if opt.ULevel>MaxULevel
      error(message('Robust:analysis:WCGAIN3',num2str(MaxULevel,'%.2g')));
   end

   % Compute stability margin
   [WCG,Info,IRD] = wcgain_(sys,opt);
catch ME
   throw(ME)
end

% Show report
sysTimeUnit = sys.TimeUnit;
if DisplayFlag
   localShowReport(WCG,Info,IRD,opt,sysTimeUnit)
end

% Construct WCU
if nout>1
   % Convert low-level ltipack.ssdata into @ss
   WCU = ss.makeS(IRD.WCU,sysTimeUnit);
end

% Construct INFO
if nout>2
   % Wrap ltipack.ssdata containers into @ss objects
   Info.WorstPerturbation = ss.makeS(Info.WorstPerturbation,sysTimeUnit);
   % Add fields BadUncertainValues and ArrayIndex for backward
   % compatibility
   Info.BadUncertainValues = Info.WorstPerturbation;
   Info.ArrayIndex = Info.Model;
end

% Backward compatibility with < 16b
if MaxOverFreqOff
   Ts = sys.Ts;
   w = Info.Frequency;
   if w(end)==Inf
      w(end) = 1e8*w(end-1);
   end
   WCG.LowerBound = frd(Info.Bounds(:,1),w,...
      'Ts',Ts,'TimeUnit',sysTimeUnit);
   WCG.UpperBound = frd(Info.Bounds(:,2),w,...
      'Ts',Ts,'TimeUnit',sysTimeUnit);
end


%--------------- Local functions ----------------------------

function [WCG,WCU,Info] = localVectorizedCall(sys,varargin)
% Supports vectorized call prior to 16b (equivalent ot FOR loop over
% models)
AS = getArraySize(sys);
for ct=prod(AS):-1:1
   [WCG(ct),WCU(ct),Info(ct)] = wcgain(subparen(sys,{':',':',ct}),varargin{:});
end
WCG = reshape(WCG,AS);
WCU = reshape(WCU,AS);
Info = reshape(Info,AS);


function localShowReport(WCG,Info,IRD,Opt,TimeUnit)
freq = WCG.CriticalFrequency;
funits = ['rad/' TimeUnit];
ulevel = round(100*Opt.ULevel);
if ~IRD.StableFlag
   if ~IRD.hasID
      disp(getString(message('Robust:analysis:WCGAINUnstableNom')))
   else
      disp(getString(message('Robust:analysis:WCGAINUnstableNomPade', int2str(Opt.PadeN))))
   end
elseif isempty(IRD.muB.BlkInfo)
   disp(getString(message('Robust:analysis:WCGAIN2',...
      num2str(WCG.LowerBound,'%0.3g'))))
elseif isinf(WCG.LowerBound)
   disp(getString(message('Robust:analysis:WCGAINInf1')))
   m1 = getString(message('Robust:analysis:WCGAINInf3'));
   disp([' -- ' m1])
   m2 = getString(message('Robust:analysis:WCGAINInf5'));
   disp([' -- ' m2])
elseif isinf(WCG.UpperBound)
   disp(getString(message('Robust:analysis:WCGAINInf2',...
      num2str(WCG.LowerBound,'%0.3g'),num2str(freq,'%0.3g'),funits)))
   m1 = getString(message('Robust:analysis:WCGAINInf4'));
   disp([' -- ' m1])
   m2 = getString(message('Robust:analysis:WCGAINInf5'));
   disp([' -- ' m2])
else
   disp(getString(message('Robust:analysis:WCGAINFinite1',...
      num2str(WCG.UpperBound,'%0.3g'))))
   m1 = getString(message('Robust:analysis:WCGAINFinite2',...
      num2str(ulevel,'%d')));
   disp([' -- ' m1])
   m2 = getString(message('Robust:analysis:WCGAINFinite3',...
      num2str(WCG.LowerBound,'%0.3g'),num2str(freq,'%0.3g'),funits));
   disp([' -- ' m2])
   if strcmp(Opt.Sensitivity,'on')
      disp(rctutil.makeSensMessage(Info.Sensitivity,2))
   end
end