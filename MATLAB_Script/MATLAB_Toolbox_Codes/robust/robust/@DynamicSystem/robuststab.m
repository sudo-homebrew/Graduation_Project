function [varargout] = robuststab(sys,opt)
%ROBUSTSTAB   Robust stability margins of uncertain systems.
%
%   [STABMARG,DESTABUNC,REPORT,INFO] = ROBUSTSTAB(SYS) calculates robust
%   stability margins for the uncertain system SYS. A robust stability  
%   margin greater than 1 means that the uncertain system is stable for 
%   all values of its modeled uncertainty. A robust stability margin less 
%   than 1 implies that some values of the uncertain elements within the 
%   specified ranges lead to instability. 
%
%   More precisely, a margin of 0.5 implies two things: 
%     * The uncertain system remains stable as long as the uncertain  
%       element values stay within d<0.5 normalized units of their nominal 
%       values
%     * There is a destabilizing combination of uncertain element values
%       within 0.5 normalized units of their nominal values.
%   See ACTUAL2NORMALIZED for converting between actual and normalized 
%   deviations from nominal values. 
%
%   ROBUSTSTAB returns the following data:
%     STABMARG  Structure with the following fields:
%         UpperBound: Upper bound on stability margin.
%         LowerBound: Lower bound on stability margin.
%         DestabilizingFrequency: Frequency at which instability occurs
%     DESTABUNC  Structure of uncertain element values which cause
%                instability. Its fields are named after the uncertain
%                elements of SYS.
%     REPORT  Text description of robustness analysis results.
%     INFO  Structure with the following fields:
%         Sensitivity: Scalar struct of sensitivities of the stability 
%                      margins to each uncertain element.
%         Frequency: Frequency vector used for analysis.
%         BadUncertainValues: Struct array (same length as Frequency) 
%                    with destabilizing uncertain element values at  
%                    each frequency
%         MussvBnds: Upper and lower bounds from MUSSV.
%         MussvInfo: Structure of compressed data from MUSSV.
%   The stability margin is 0 if the nominal value of SYS is unstable.
%   Note that UFRD models are always assumed to be nominally stable.
%
%   [STABMARG,DESTABUNC,REPORT,INFO] = ROBUSTSTAB(SYS,OPTS) specifies the
%   options OPTS. Use ROBUSTSTABOPTIONS to create OPTS.
%
%   See also: ROBUSTSTABOPTIONS, ACTUAL2NORMALIZED, LOOPMARGIN, MUSSV, 
%   ROBUSTPERF, WCGAIN, WCSENS, WCMARGIN.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.
nin = nargin;
nout = nargout;
if nin==1
   opt = robuststabOptions;
elseif ~isa(opt,'rctoptions.robuststab')
   ctrlMsgUtils.error('Robust:obsolete:IllegalRSoption');
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

% Transform to uncertain model.  Tunable parameters are set to current
% value.
if isFRD
   usys = ufrd(sys);
else
   usys = uss(sys);
end
try
   [StabMarg,INFO,IRD] = robuststab_(usys,opt,sNeed);
catch ME
   throw(ME)
end
UBN = unique(cat(1,IRD.BlockName));  % UniqueBlockNames

% Repackage results
AS = size(StabMarg);
nsys = prod(AS);

if nout>=1
   varargout{1} = StabMarg;
end

if nout>=2
   % DESTABUNC is a struct array, fields are the union of all of the block
   % names across the input array.  Any values of class SSDATA need to be
   % recast as SS, and the TimeUnit property must be set to match the input
   % system's TimeUnit.
   DESTABUNC = cell2struct(cell([numel(UBN) AS]),UBN,1);
   for i=1:nsys
      BN = IRD(i).BlockName;
      DSV = ss.makeS(IRD(i).DestabilizingValue,sysTimeUnit);
      for j=1:numel(BN)
         DESTABUNC(i).(BN{j}) = DSV.(BN{j});
      end
   end
   varargout{2} = DESTABUNC;
end

if nout>=3
   REPORT = repmat(' ',[1 1 AS]);
   for i=1:nsys
      R = LOCALmakereport(isFRD,sysTimeUnit,opt,IRD(i),StabMarg(i));
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
function R = LOCALmakereport(FRDModel,TimeUnit,Opt,IRD,SM)
NotStandard = ~IRD.muB.ismuStandard;
BlkInfo = IRD.muB.BlkInfo;
if NotStandard
   R = ctrlMsgUtils.message('Robust:obsolete:IllegalDescription');
elseif ~IRD.StableFlag  % && ~FRDModel
   if ~IRD.hasID
      R = ctrlMsgUtils.message('Robust:analysis:USSUnstableNomRS');
   else
      R = ctrlMsgUtils.message('Robust:analysis:USSUnstableNomPadeRS', int2str(Opt.PadeN));
   end
elseif isempty(BlkInfo)
   if ~FRDModel
      R = ctrlMsgUtils.message('Robust:obsolete:USSNoBlocksRS');
   else
      R = ctrlMsgUtils.message('Robust:obsolete:UFRDNoBlocksRS');
   end
else
   if ~FRDModel
      if SM.UpperBound <= 1
         RHeader = ctrlMsgUtils.message('Robust:analysis:NotRS');
      elseif SM.LowerBound < 1
         RHeader = ctrlMsgUtils.message('Robust:analysis:PossibleRS');
      else
         RHeader = ctrlMsgUtils.message('Robust:analysis:YesRS');
      end
   else
      if SM.UpperBound <= 1
         RHeader = ctrlMsgUtils.message('Robust:analysis:NotRS');
      elseif SM.LowerBound < 1
         RHeader = ctrlMsgUtils.message('Robust:obsolete:UFRDPossibleRS');
      else
         RHeader = ctrlMsgUtils.message('Robust:obsolete:UFRDIsRS');
      end
   end
   freq = SM.DestabilizingFrequency;
   funits = ['rad/' TimeUnit];
   if ~isempty(BlkInfo)
      if isinf(SM.LowerBound) % pku==0
         text1 = [' -- ' ctrlMsgUtils.message('Robust:analysis:InfiniteRSMargin')];
         text2 = '';
         text3 = '';
      else
         PA = 100*SM.LowerBound;
         m1 = ctrlMsgUtils.message('Robust:analysis:TolerateRSMargin',[num2str(PA,'%0.3g') '%']); % %
         text1 = [' -- ' m1];
         if isinf(SM.UpperBound) % pkl==0
            m2 = ctrlMsgUtils.message('Robust:analysis:NoBadUncertaintyFoundRS');
            text2 = [' --   ' m2];
            text3 = '';
         else
            PA = 100*SM.UpperBound;
            m2 = ctrlMsgUtils.message('Robust:analysis:BadUncertaintyFoundRS',[num2str(PA,'%0.3g') '%']); % %
            text2 = [' -- ' m2]; % %
            m3 = ctrlMsgUtils.message('Robust:analysis:BadUncertaintyCausesRS',num2str(freq,'%0.3g'),funits);
            text3 = [' -- ' m3];
         end
      end
      RHeader = char(RHeader,text1,text2,text3);
   end
   R = char(RHeader,IRD.SensText);
end
