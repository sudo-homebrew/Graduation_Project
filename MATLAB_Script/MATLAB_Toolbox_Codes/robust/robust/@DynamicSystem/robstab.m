function [StabMarg,WCU,Info] = robstab(sys,varargin)
%ROBSTAB   Robust stability of uncertain system.
%
%   [STABMARG,WCU] = ROBSTAB(USYS) calculates the robust stability margin
%   for the uncertain system USYS (USS or UFRD). This margin is relative to
%   the  uncertainty level specified in USYS. A robust stability margin
%   greater than 1 means that USYS is stable for all values of its modeled
%   uncertainty. A robust stability margin less than 1 implies that USYS
%   becomes unstable for some values of the uncertain elements within their
%   specified ranges. For example, a margin of 0.5 implies that
%     * USYS remains stable as long as the uncertain element values stay
%       strictly within 0.5 normalized units of their nominal values
%     * There is a destabilizing perturbation of size 0.5 normalized units.
%   Use NORMALIZED2ACTUAL or USCALE to convert normalized deviations from
%   nominal values to actual uncertainty ranges. The stability margin is 0
%   if the nominal value of USYS is unstable. Note that UFRD models are
%   always assumed to be nominally stable.
%
%   ROBSTAB returns the following data:
%     STABMARG  Structure with fields:
%          LowerBound: Lower bound on stability margin
%          UpperBound: Upper bound on stability margin
%          CriticalFrequency: Frequency (in radians/TimeUnit) where 
%                             stability margin is smallest.
%     WCU  Smallest perturbation of uncertain elements causing instability.
%          WCU is a structure containing (actual) perturbation values for 
%          each uncertain element of USYS. Use USUBS(USYS,WCU) to apply
%          this perturbation to the nominal system.
%
%   [...] = ROBSTAB(USYS,{WMIN,WMAX}) restricts the robust stability margin
%   computation to the frequency interval [WMIN,WMAX].
%
%   [...] = ROBSTAB(USYS,W) specifies a vector W of frequency points
%   (in radians/TimeUnit) where to compute the robust stability margin.
%
%   [...] = ROBSTAB(USYS,...,OPTIONS) specifies additional options, see 
%   robOptions for details.
%
%   [STABMARG,WCU,INFO] = ROBSTAB(USYS,...) returns a structure INFO with
%   the following fields:
%      Model   When USYS is an array of models, index of model with weakest
%              robust stability margin
%      Frequency  N-by-1 vector of frequency points
%      Bounds  N-by-2 array of lower and upper bounds for the stability
%              margin at each frequency point
%      WorstPerturbation  N-by-1 structure of smallest destabilizing 
%              perturbations at each frequency point
%      Sensitivity  STABMARG sensitivity (in percent) to each uncertain 
%              element (when the "Sensitivity" option is set to 'on').
%
%   Example: Consider a second-order plant with 20% relative uncertainty:
%      delta = ultidyn('delta',[1 1]); 
%      G = tf(1,[1 .2 1]) * (1+0.2*delta);
%   Close the loop on a PID controller and analyze the robust stability
%   of the closed-loop system:
%      C = pid(1,2,0.9,0.1);
%      S = feedback(1,G*C); 
%      [SM,WCU] = robstab(S);
%   The robust stability margin is 0.84, so the closed-loop system can only
%   withstand 84% of the specified uncertainty without going unstable.
%   Confirm WCU causes instability by setting the uncertain elements to 
%   the values in WCU:
%      pole(usubs(S,WCU))
%   The size of WCU in normalized units coincides with SM.UpperBound:
%      getPeakGain(actual2normalized(delta,WCU.delta))
%      SM.UpperBound
%
%   See also ROBOPTIONS, USCALE, NORMALIZED2ACTUAL, ACTUAL2NORMALIZED, ROBGAIN, 
%   WCGAIN, DISKMARGIN, WCDISKMARGIN, MUSSV, USS, UFRD.

%   Author(s): MUSYN, P. Gahinet
%   Copyright 2004-2016 The MathWorks, Inc.
narginchk(1,3)
nin = nargin;
nout = nargout;
% Get options
if nin>1 && ~(isnumeric(varargin{end}) || iscell(varargin{end}))
   opt = varargin{end};  nin = nin-1;
   if isa(opt,'rctoptions.robuststab')
      % Remap
      opt = robOptions('Display',opt.Display,'Sensitivity',opt.Sensitivity,...
         'SensitivityPercent',opt.VaryUncertainty,'MussvOptions',opt.Mussv);
   elseif ~isa(opt,'rctoptions.robAnalysis')
      error(message('Robust:analysis:ROBSTAB2'))
   end
else
   opt = robOptions();
end
DisplayFlag = strcmp(opt.Display,'on');

try
   % Convert SYS to USS or UFRD, validate options
   [sys,opt] = robSetUp(sys,opt,varargin{1:nin-1});
   % Adjust options to eliminate wasteful computation
   if nout<3
      opt.VaryFrequency = 'off';
      if ~DisplayFlag
         opt.Sensitivity = 'off';
      end
   end
   % Compute stability margin
   [StabMarg,Info,IRD] = robstab_(sys,opt);
catch ME
   throw(ME)
end

% Show report
sysTimeUnit = sys.TimeUnit;
if DisplayFlag
   localShowReport(StabMarg,Info,IRD,opt,sysTimeUnit)
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
function localShowReport(SM,Info,IRD,Opt,TimeUnit)
if ~IRD.StableFlag
   if ~IRD.hasID
      disp(getString(message('Robust:analysis:USSUnstableNomRS')))
   else
      disp(getString(message('Robust:analysis:USSUnstableNomPadeRS', int2str(Opt.PadeN))))
   end
elseif isempty(IRD.muB.BlkInfo)
   disp(getString(message('Robust:analysis:ROBSTAB5')))
else
   if SM.UpperBound <= 1
      disp(getString(message('Robust:analysis:NotRS')))
   elseif SM.LowerBound < 1
      disp(getString(message('Robust:analysis:PossibleRS')))
   else
      disp(getString(message('Robust:analysis:YesRS')))
   end
   freq = SM.CriticalFrequency;
   funits = ['rad/' TimeUnit];
   if isinf(SM.LowerBound) % pku==0
      disp([' -- ' getString(message('Robust:analysis:InfiniteRSMargin'))])
   else
      PA = 100*SM.LowerBound;
      m1 = getString(message('Robust:analysis:TolerateRSMargin',[num2str(PA,'%0.3g') '%'])); % %
      disp([' -- ' m1])
      if isinf(SM.UpperBound) || IRD.OutOfRangeUREAL
         % No destabilizing perturbation found, or destabilizing
         % perturbation outside UREAL valid range
         m2 = getString(message('Robust:analysis:NoBadUncertaintyFoundRS'));
         disp([' -- ' m2])
      else
         PA = 100*SM.UpperBound;
         m2 = getString(message('Robust:analysis:BadUncertaintyFoundRS',[num2str(PA,'%0.3g') '%'])); % %
         disp([' -- ' m2])
         m3 = getString(message('Robust:analysis:BadUncertaintyCausesRS',num2str(freq,'%0.3g'),funits));
         disp([' -- ' m3])
      end
   end
   if strcmp(Opt.Sensitivity,'on')
      disp(rctutil.makeSensMessage(Info.Sensitivity,1))
   end
end
