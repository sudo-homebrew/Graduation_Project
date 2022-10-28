function [PerfMarg,WCU,Info] = robgain(sys,gamma,varargin)
%ROBGAIN   Robust performance of uncertain system.
%
%   [PERFMARG,WCU] = ROBGAIN(USYS,GAMMA) calculates the robust performance
%   margin for the uncertain system USYS and the performance level GAMMA.
%   Here performance is measured by the peak input/output gain of USYS
%   (H-infinity norm). The performance margin is relative to the uncertainty
%   level specified in USYS (USS or UFRD model). A margin greater than 1
%   means that the gain of USYS remains below GAMMA for all values of its
%   modeled uncertainty. A margin less than 1 implies that at some frequency,
%   the gain of USYS exceeds GAMMA for some values of the uncertain elements
%   within their specified ranges. For example, a margin of 0.5 implies that
%     * The gain of USYS is less than GAMMA as long as the uncertain element
%       values stay within 0.5 normalized units of their nominal values
%     * There is a perturbation of size 0.5 normalized units that drives the
%       peak gain of USYS to GAMMA.
%   Use NORMALIZED2ACTUAL or USCALE to convert normalized deviations from 
%   nominal values to actual uncertainty ranges. The performance margin is 
%   zero if the nominal value of USYS is unstable or its peak gain exceeds 
%   GAMMA. Note that UFRD models are always assumed to be nominally stable.
%
%   ROBGAIN returns the following data:
%     PERFMARG  Structure with fields:
%          LowerBound: Lower bound on performance margin
%          UpperBound: Upper bound on performance margin
%          CriticalFrequency: Frequency (in radians/TimeUnit) where
%                             performance margin is smallest.
%     WCU  Smallest perturbation of uncertain elements driving the peak gain
%          to GAMMA. WCU is a structure containing (actual) perturbation
%          values for each uncertain element of USYS. Use USUBS(USYS,WCU)
%          to apply this perturbation to the nominal system.
%
%   [...] = ROBGAIN(USYS,GAMMA,{WMIN,WMAX}) assesses the robust performance
%   margin in the frequency interval [WMIN,WMAX]. Gain values at frequencies
%   outside this interval are ignored.
%
%   [...] = ROBGAIN(USYS,GAMMA,W) specifies a vector W of frequency points
%   (in radians/TimeUnit) where to evaluate gains and compute the robust 
%   performance margin.
%
%   [...] = ROBGAIN(USYS,GAMMA,...,OPTIONS) specifies additional options,
%   see robOptions for details.
%
%   [PERFMARG,WCU,INFO] = ROBGAIN(USYS,GAMMA,...) returns a structure INFO
%   with the following fields:
%      Model   When USYS is an array of models, index of model with weakest
%              robust performance margin
%      Frequency  N-by-1 vector of frequency points
%      Bounds  N-by-2 array of lower and upper bounds for the performance
%              margin at each frequency point (relative to GAMMA)
%      WorstPerturbation  N-by-1 structure of smallest perturbations needed
%              at each frequency point to drive the gain to GAMMA
%      Sensitivity  PERFMARG sensitivity (in percent) to each uncertain 
%              element (when the "Sensitivity" option is set to 'on').
%
%   Example: Consider a second-order plant with 10% relative uncertainty:
%      delta = ultidyn('delta',[1 1]); 
%      G = tf(1,[1 .6 1]) * (1+0.1*delta);
%   Close the loop on a PID controller and check whether the closed-loop 
%   sensitivity can exceed 1.1:
%      C = pid(2.3,1,1.3,0.01);
%      S = feedback(1,G*C);   % sensitivity function
%      [PM,WCU] = robgain(S,1.1);
%   The robust performance margin is 0.62, so it takes only 62% of the 
%   specified uncertainty to drive the peak sensitivity above 1.1. Set
%   the uncertain elements to the values in WCU and confirm that the peak
%   sensitivity is 1.1 for WCU:
%      WCS = usubs(S,WCU);
%      getPeakGain(WCS,1e-6)
%   The size of WCU in normalized units coincides with SM.UpperBound:
%      getPeakGain(actual2normalized(delta,WCU.delta))
%      PM.UpperBound
%
%   See also ROBOPTIONS, USCALE, NORMALIZED2ACTUAL, ACTUAL2NORMALIZED, ROBSTAB, 
%   WCGAIN, MUSSV, USS, UFRD.

%   Author(s): MUSYN, P. Gahinet
%   Copyright 2004-2016 The MathWorks, Inc.
narginchk(2,4)
nin = nargin;
nout = nargout;
% Check GAMMA
if ~(isnumeric(gamma) && isscalar(gamma) && isreal(gamma) && gamma>0)
   error(message('Robust:analysis:ROBGAIN2'))
end
% Get options
if nin>2 && ~(isnumeric(varargin{end}) || iscell(varargin{end}))
   opt = varargin{end};  nin = nin-1;
   if isa(opt,'rctoptions.robustperf')
      % Remap
      opt = robOptions('Display',opt.Display,'Sensitivity',opt.Sensitivity,...
         'SensitivityPercent',opt.VaryUncertainty,'MussvOptions',opt.Mussv);
   elseif ~isa(opt,'rctoptions.robAnalysis')
      error(message('Robust:analysis:ROBGAIN1'))
   end
else
   opt = robOptions();
end
DisplayFlag = strcmp(opt.Display,'on');

try
   % Convert SYS to USS or UFRD, validate options
   [sys,opt] = robSetUp(sys,opt,varargin{1:nin-2});
   % Adjust options to eliminate wasteful computation
   if nout<3
      opt.VaryFrequency = 'off';
      if ~DisplayFlag
         opt.Sensitivity = 'off';
      end
   end
   % Compute stability margin
   if isinf(gamma)
      [PerfMarg,Info,IRD] = robstab_(sys,opt);
   else
      [PerfMarg,Info,IRD] = robgain_(sys,gamma,opt);
   end
catch ME
   throw(ME)
end

% Show report
sysTimeUnit = sys.TimeUnit;
if DisplayFlag
   localShowReport(PerfMarg,Info,gamma,IRD,opt,sysTimeUnit)
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
end

%--------------- Local functions ----------------------------

function localShowReport(RGM,Info,gamma,IRD,Opt,TimeUnit)
gStr = num2str(gamma,'%0.6g');
if ~IRD.StableFlag
   if ~IRD.hasID
      disp(getString(message('Robust:analysis:USSUnstableNomRP')))
   else
      disp(getString(message('Robust:analysis:USSUnstableNomPadeRP', int2str(Opt.PadeN))))
   end
elseif IRD.NominalPerf>=gamma
   disp(getString(message('Robust:analysis:ROBGAIN3',gStr)))
elseif isempty(IRD.muB.BlkInfo)
   disp(getString(message('Robust:analysis:ROBSTAB5')))
else
   if RGM.UpperBound <= 1
      disp(getString(message('Robust:analysis:NotRG', gStr)))
   elseif RGM.LowerBound < 1
      disp(getString(message('Robust:analysis:PossibleRG', gStr)))
   else
      disp(getString(message('Robust:analysis:YesRG', gStr)))
   end
   freq = RGM.CriticalFrequency;
   funits = ['rad/' TimeUnit];
   if isinf(RGM.LowerBound) % pku==0
      disp([' -- ' getString(message('Robust:analysis:InfiniteRGMargin',gStr))])
   else
      LA = 100*RGM.LowerBound;
      m1 = getString(message('Robust:analysis:guaranteedGainRG',...
         [num2str(LA,'%0.3g') '%'],num2str(gamma)));
      disp([' -- ' m1])
      if isinf(RGM.UpperBound) || IRD.OutOfRangeUREAL
         % No performance-busting perturbation found, or such perturbation 
         % outside UREAL valid range
         m2 = getString(message('Robust:analysis:NoBadUncertaintyFoundRG',gStr));
         disp([' -- ' m2])
      else
         PA = 100*RGM.UpperBound;
         m2 = getString(message('Robust:analysis:BadUncertaintyFoundRG',...
            [num2str(PA,'%0.3g') '%'])); % %
         disp([' -- ' m2])
         m3 = getString(message('Robust:analysis:BadUncertaintyCausesRG',...
            gStr,num2str(freq,'%0.3g'),funits)); % %
         disp([' -- ' m3])
      end
   end
   if strcmp(Opt.Sensitivity,'on')
      disp(rctutil.makeSensMessage(Info.Sensitivity,1))
   end
end