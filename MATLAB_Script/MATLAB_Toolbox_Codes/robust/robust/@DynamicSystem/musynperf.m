function [BND,WCU,Info] = musynperf(sys,varargin)
%MUSYNPERF   Robust H-infinity performance optimized by MUSYN.
%
%   The robust H-infinity norm of the uncertain closed-loop system CLP is
%   the smallest GAM such that the I/O gain of CLP stays below GAM for all
%   modeled uncertainty up to size 1/GAM (in normalized units). For example,
%   a value of GAM=1.125 implies two things:
%     * For all values of uncertain elements less than 0.8 normalized units
%       from their nominal values, the input/output gain of CLP remains
%       less than 1.125
%     * The peak I/O gain of CLP reaches 1.125 for some combination of
%       uncertain element values within 0.8 normalized units of their
%       nominal values.
%   Use NORMALIZED2ACTUAL or USCALE to convert normalized deviations from 
%   nominal values to actual uncertainty ranges. MUSYN minimizes the robust
%   H-infinity performance GAM over all possible choices of controller.
%   MUSYNPERF computes the robust H-infinity performance GAM given the
%   uncertain model CLP.
%
%   [BND,WCU] = MUSYNPERF(CLP) takes a USS or UFRD model CLP and returns
%   the following data:
%     BND  Structure with fields:
%          LowerBound: Lower bound on robust performance GAM
%          UpperBound: Upper bound on robust performance GAM
%          CriticalFrequency: Frequency (in radians/TimeUnit) where the
%                             I/O gain of CLP reaches BND.LowerBound
%     WCU  Combination of uncertain element values for which the I/O gain
%          of CLP reaches BND.LowerBound. WCU is a structure containing
%          actual values for each uncertain element of CLP. Use
%          USUBS(CLP,WCU) to apply these values to the nominal system.
%
%   [...] = MUSYNPERF(CLP,{WMIN,WMAX}) restricts the H-infinity norm
%   computation to the frequency interval [WMIN,WMAX].
%
%   [...] = MUSYNPERF(CLP,W) specifies a vector W of frequency points
%   (in radians/TimeUnit) where to compute the robust I/O gain.
%
%   [...] = MUSYNPERF(CLP,...,OPTIONS) specifies additional options, see
%   robOptions for details.
%
%   [BND,WCU,INFO] = MUSYNPERF(CLP,...) returns a structure INFO with the
%   following fields:
%      Frequency  N-by-1 vector of frequency points
%      Bounds  N-by-2 array of lower and upper bounds for robust I/O gain
%              at each frequency point
%      WorstPerturbation  N-by-1 structure of associated perturbations at
%              each frequency point
%      Sensitivity  Sensitivity (in percent) of robust H-infinity norm GAM
%              to each uncertain element (when the "Sensitivity" option is
%              set to 'on').
%
%   Example: Use REDUCE and MUSYNPERF to simplify the controller K returned
%   by MUSYN. Start by computing reduced-order controllers for orders
%   ranging from 1 to full order:
%       N = order(K);
%       Kred = reduce(K,1:N);
%   If CLPERF is the robust H-infinity norm achieved by MUSYN (third output
%   argument), find the lowest-order controller KLOW with performance no 
%   worse than 1.05*CLPERF (5% degradation):
%       for k=1:N
%          Klow = Kred(:,:,k);
%          bnd = musynperf(lft(P,Klow));
%          if bnd.UpperBound < 1.05*CLPERF
%             break
%          end
%       end
%
%   See also musyn, robOptions, uscale, normalized2actual, actual2normalized,
%   robgain, robstab, wcgain, diskmargin, wcdiskmargin, mussv, uss, ufrd.

%   Copyright 2003-2016 The MathWorks, Inc.
narginchk(1,3)
nin = nargin;
nout = nargout;

% Single model only
if nmodels(sys)>1
   error(message('Control:general:RequiresSingleModel','musynperf'))
end

% Get options
if nin>1 && ~(isnumeric(varargin{end}) || iscell(varargin{end}))
   opt = varargin{end};  nin = nin-1;
   if ~isa(opt,'rctoptions.robAnalysis')
      error(message('Robust:analysis:DKSYNPERF1'))
   end
else
   opt = robOptions();
end
DisplayFlag = strcmp(opt.Display,'on');

try
   % Convert SYS to USS or UFRD, validate options
   [sys,opt] = robSetUp(sys,opt,varargin{1:nin-1});
   % Add performance block
   ios = iosize(sys);
   perfblk = ultidyn('RCTReservedName_',ios([2 1]));
   perfblk.AutoSimplify = 'off';   % watch for auto-simplification in 0x0 system
   sys = lft(sys,perfblk);
   % Adjust options to eliminate wasteful computation
   if nout<3
      opt.VaryFrequency = 'off';
      if ~DisplayFlag
         opt.Sensitivity = 'off';
      end
   end
   % Compute stability margin
   [SM,Info,IRD] = robstab_(sys,opt);
   % Package data
   BND = struct(...
      'LowerBound',1/SM.UpperBound,...
      'UpperBound',1/SM.LowerBound,...
      'CriticalFrequency',SM.CriticalFrequency);
   WCU = rmfield(IRD.WCU,'RCTReservedName_');
   Info = rmfield(Info,'Model');
   Info.Bounds = 1./Info.Bounds(:,[2 1]);
   Info.WorstPerturbation = rmfield(Info.WorstPerturbation,'RCTReservedName_');
   Info.Sensitivity = rmfield(Info.Sensitivity,'RCTReservedName_');
catch ME
   throw(ME)
end

% Show report
sysTimeUnit = sys.TimeUnit;
if DisplayFlag
   localShowReport(BND,Info,IRD,opt,sysTimeUnit)
end

% Construct WCU
if nout>1
   % Convert low-level ltipack.ssdata into @ss
   WCU = ss.makeS(WCU,sysTimeUnit);
end

% Construct INFO
if nout>2
   % Wrap ltipack.ssdata containers into @ss objects
   Info.WorstPerturbation = ss.makeS(Info.WorstPerturbation,sysTimeUnit);
end

%--------------- Local functions ----------------------------
function localShowReport(muBND,Info,IRD,Opt,TimeUnit)
if ~IRD.StableFlag
   if ~IRD.hasID
      disp(getString(message('Robust:analysis:DKSYNPERF2')))
   else
      disp(getString(message('Robust:analysis:DKSYNPERF3', int2str(Opt.PadeN))))
   end
elseif isscalar(IRD.muB.BlkInfo)
   disp(getString(message('Robust:analysis:DKSYNPERF4', num2str(muBND.LowerBound,'%.3g'))))
elseif muBND.UpperBound==0
   disp(getString(message('Robust:analysis:DKSYNPERF5')))
else
   disp(getString(message('Robust:analysis:DKSYNPERF6',...
      num2str(muBND.UpperBound,'%.3g'))))
   m1 = getString(message('Robust:analysis:DKSYNPERF7',...
      num2str(muBND.UpperBound,'%.3g'),...
      num2str(1/muBND.UpperBound,'%.3g')));
   disp([' -- ' m1])
   if muBND.LowerBound==0 || IRD.OutOfRangeUREAL
      m2 = getString(message('Robust:analysis:DKSYNPERF8'));
      disp([' -- ' m2])
   else
      funits = ['rad/' TimeUnit];
      m2 = getString(message('Robust:analysis:DKSYNPERF9',...
         num2str(1/muBND.LowerBound,'%.3g')));
      m3 = getString(message('Robust:analysis:DKSYNPERF10',...
         num2str(muBND.LowerBound,'%.3g'),...
         num2str(muBND.CriticalFrequency,'%0.3g'),funits));
      disp([' -- ' m2])
      disp([' -- ' m3])
   end
   if strcmp(Opt.Sensitivity,'on')
      % The email has a suggestion as to what the sensitivity language
      % should be, and it will require either a separate makeSensMessage or
      % flag to specify which type of message you want (eg., robstab, or
      % musynperf)
      disp(rctutil.makeSensMessage(Info.Sensitivity,3))
   end
end