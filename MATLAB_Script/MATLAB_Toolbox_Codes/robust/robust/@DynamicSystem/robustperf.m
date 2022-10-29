function [varargout] = robustperf(sys,opt)
%ROBUSTPERF   Robust performance margins of uncertain systems.
%
%   [PERFMARG,PERFMARGUNC,REPORT,INFO] = ROBUSTPERF(SYS) calculates the
%   robust performance margin of the uncertain system SYS. Performance is
%   measured by the input/output gain of SYS. This gain will generally
%   increase with uncertainty. ROBUSTPERF computes the Robust Performance 
%   Margin, a measure of performance degradation in the face of uncertainty.
%   A robust performance margin greater than 1 means that the input/output 
%   gain (H-infinity norm) of SYS remains less than 1 for all values of 
%   the modeled uncertainty. A robust performance margin less than 1 implies 
%   that some values of the uncertain elements within the specified ranges 
%   lead to an input/output gain greater than 1.
%
%   More precisely, a margin of 0.4 implies two things: 
%     * For all values of uncertain elements less than 0.4 normalized units 
%       from their nominal values, the input/output gain remains less than 
%       1/0.4=2.5
%     * There is a combination of uncertain element values within 0.4 
%       normalized units of their nominal values that achieves the 
%       gain value of 2.5.
%   See ACTUAL2NORMALIZED for converting between actual and normalized 
%   deviations from the nominal value of an uncertain element.
%
%   ROBUSTPERF returns the following data:
%     PERFMARG  Structure with the following fields:
%        UpperBound: Upper bound on performance margin.
%        LowerBound: Lower bound on performance margin.
%        CriticalFrequency: Frequency of worst performance degradation
%     PERFMARGUNC  Structure of uncertain element values leading to
%                  worst performance degradation. Its fields are named 
%                  after the uncertain elements of SYS.
%     REPORT  Text description of robust performance results.
%     INFO  Structure with the following fields:
%        Sensitivity: Scalar struct of sensitivities of the performance 
%                     margins to each uncertain element.
%        Frequency: Frequency vector associated with analysis.
%        BadUncertainValues: Struct array (same length as Frequency) with 
%                   worst values of uncertain elements at each frequency
%        MussvBnds: Upper and lower bounds from MUSSV.
%        MussvInfo: Structure of compressed data from MUSSV.
%   The performance margin is 0 if the nominal value of SYS is unstable.
%   Note that UFRD models are always assumed to be nominally stable.
%
%   [PERFMARG,PERFMARGUNC,REPORT,INFO] = ROBUSTPERF(SYS,OPTS) specifies the
%   options OPTS. Use ROBUSTPERFOPTIONS to create OPTS.
%
%   See also: ROBUSTPERFOPTIONS, ACTUAL2NORMALIZED, LOOPMARGIN, MUSSV, 
%   ROBUSTSTAB, WCGAIN, WCSENS, WCMARGIN.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.
nin = nargin;
nout = nargout;
if nin==1
   opt = robustperfOptions;
elseif ~isa(opt,'rctoptions.robuststab')
   ctrlMsgUtils.error('Robust:obsolete:IllegalRPoption');
end
sysTimeUnit = sys.TimeUnit;
isFRD = isa(sys,'FRDModel');

% Decide what is needed to compute, based on NARGOUT and OPT
sNeed.Report = (nout>=3);  % 3rd output argument
sNeed.Sensitivity = (strcmpi(opt.Sensitivity,'on') && sNeed.Report);
sNeed.AllBadUncertainValues = (nout==4);  % these are in INFO
sNeed.Mussv4Info = (nout==4);  % these are in INFO

% Two option fields (.Mussv and .Display) compete for whether the MUSSV
% "counter" should display the points.  Decision here is that .Display wins
% this battle, and we make consistent.   Default for .Display is 'off'.
if strcmp(opt.Display,'on')
   opt.Mussv('s'==opt.Mussv) = [];
else
   opt.Mussv = [opt.Mussv 's'];
end

% Transform to uncertain model. Tunable parameters are set to current
% value.
if isFRD
   usys = ufrd(sys);
else
   usys = uss(sys);
end
try
   [PerfMarg,INFO,IRD] = robustperf_(usys,opt,sNeed);
catch ME
   throw(ME)
end
UBN = unique(cat(1,IRD.BlockName));  % UniqueBlockNames

% Repackage results
AS = size(PerfMarg);
nsys = prod(AS);

if nout>=1
   varargout{1} = PerfMarg;
end

if nout>=2
   % PERFMARGUNC is a struct array, fields are the union of all of the block
   % names across the input array.  Any values of class SSDATA need to be
   % recast as SS, and the TimeUnit property must be set to match the input
   % system's TimeUnit.
   PERFMARGUNC = cell2struct(cell([numel(UBN) AS]),UBN,1);
   for i=1:nsys
      BN = IRD(i).BlockName;
      DSV = ss.makeS(IRD(i).DestabilizingValue,sysTimeUnit);
      for j=1:numel(BN)
         PERFMARGUNC(i).(BN{j}) = DSV.(BN{j});
      end
   end
   varargout{2} = PERFMARGUNC;
end

if nout>=3
   REPORT = repmat(' ',[1 1 AS]);
   for i=1:nsys
      R = LOCALmakereport(isFRD,sysTimeUnit,opt,IRD(i),PerfMarg(i));
      sR = size(R);
      REPORT(1:sR(1),1:sR(2),i) = R;
   end
   varargout{3} = REPORT;
end


if nout>=4
   % Wrap ltipack.ssdata containers into @ss objects
   if isFRD
      FU = sys.FrequencyUnit;
   else
      FU = 'rad/TimeUnit';
   end
   for i=1:nsys
      INFO(i).BadUncertainValues = ss.makeS(INFO(i).BadUncertainValues,sysTimeUnit);
      INFO(i).MussvBnds = set(frd.make(INFO(i).MussvBnds),'TimeUnit',sysTimeUnit,...
         'FrequencyUnit',FU);
      INFO(i).MussvInfo = frd.makeS(INFO(i).MussvInfo,sysTimeUnit,FU);
   end
   varargout{4} = INFO;
end

%--------------- Local functions ----------------------------
function R = LOCALmakereport(FRDModel,TimeUnit,Opt,IRD,PM)
NotStandard = ~IRD.muB.ismuStandard;
BlkInfo = IRD.muB.BlkInfo;
if NotStandard
   R = ctrlMsgUtils.message('Robust:obsolete:IllegalDescription');
elseif ~IRD.StableFlag
   % Only SS-like can be flagged as UNSTABLE.
   if ~IRD.hasID
      R = ctrlMsgUtils.message('Robust:analysis:USSUnstableNomRP');
   else
      R = ctrlMsgUtils.message('Robust:analysis:USSUnstableNomPadeRP', int2str(Opt.PadeN));
   end
elseif isempty(BlkInfo)
   if ~FRDModel
      R = ctrlMsgUtils.message('Robust:obsolete:USSNoBlocksRP');
   else
      R = ctrlMsgUtils.message('Robust:obsolete:UFRDNoBlocksRP');
   end
else
   if ~FRDModel
      if PM.UpperBound <= 1
         RHeader = ctrlMsgUtils.message('Robust:obsolete:NotRP');
      elseif PM.LowerBound < 1
         RHeader = ctrlMsgUtils.message('Robust:obsolete:PossibleRP');
      else
         RHeader = ctrlMsgUtils.message('Robust:obsolete:YesRP');
      end
   else
      if PM.UpperBound <= 1
         RHeader = ctrlMsgUtils.message('Robust:obsolete:NotRP');
      elseif PM.LowerBound < 1
         RHeader = ctrlMsgUtils.message('Robust:obsolete:UFRDPossibleRP');
      else
         RHeader = ctrlMsgUtils.message('Robust:obsolete:UFRDIsRP');
      end
   end
   freq = PM.CriticalFrequency;
   funits = ['rad/' TimeUnit];
   if ~isempty(BlkInfo)
      if isinf(PM.LowerBound) % pku==0
         text1 = [' -- ' ctrlMsgUtils.message('Robust:obsolete:InfiniteRPMargin')];
         text2 = '';
      else
         PA = 100*PM.LowerBound;
         m1 = ctrlMsgUtils.message('Robust:obsolete:TolerateRPMargin',[num2str(PA,'%0.3g') '%']); % %
         text1 = [' -- ' m1];
         if isinf(PM.UpperBound) % pkl==0
            m2 = ctrlMsgUtils.message('Robust:obsolete:NoBadUncertaintyFoundRP');
            text2 = [' --   ' m2];
         else
            PA = 100*PM.UpperBound;
            GA = 1/PM.UpperBound;
            m2 = ctrlMsgUtils.message('Robust:obsolete:BadUncertaintyFoundRP',...
               [num2str(PA,'%0.3g') '%'],num2str(GA,'%0.3g'),num2str(freq,'%0.3g'),funits); % %
            text2 = [' -- ' m2]; % %
         end
      end
      RHeader = char(RHeader,text1,text2);
   end
   R = char(RHeader,IRD.SensText);
end
