function [DataOut,AData] = wcgbnb(AData,opts)
%

%   Copyright 2011 The MathWorks, Inc.

starttime = clock;
GTH.M = 1.04;
GTH.A = 0.05;
BTH.M = opts.RelMax;
BTH.A = opts.AbsMax;
ABSTOL = opts.AbsTol;
RELTOL = opts.RelTol;
MAXTIME = opts.MaxTime;
MAXBRANCHES = opts.MaxBranches;
LBONLY = opts.LowerBoundOnly;
NTIMES = opts.NSearch;
MAXCNT = opts.MaxCnt;

npts = numel(AData);
activeidx = 1:npts;
AllScalednomgain = [AData.Scalednomgain];
AllIOScalings = [AData.IOScalings];
[ActLb,LbIdx] = max( [AData.ScaledLb]./AllIOScalings );
ActUb = max( [AData.ScaledUb]./AllIOScalings );

BestActLb = ([AData.ScaledUb] - [AData.agap]) ./ AllIOScalings;
ABSTOL = ABSTOL+(ActUb-max(BestActLb));
RELTOL = (1+RELTOL)*(ActUb/max(BestActLb)) -  1;

% Clean up: go through each element, and delete cubes with upper
%   bounds that are lower than the current best LOWER bound
for ii=1:length(activeidx)
   i = activeidx(ii);
   ACTIVE = AData(i).cubeidx.ACTIVE;
   UPPER = AData(i).cubeidx.UPPER;
   useless = AData(i).cubelist(:,ACTIVE)==1 & AData(i).cubelist(:,UPPER)<AllIOScalings(i)*ActLb;
   % deletecube (cube j in element i)
   AData(i).cubelist(useless,ACTIVE) = 0;
   % delete element
   if all(AData(i).cubelist(:,ACTIVE)==0)
      activeidx(ii) = 0;
   end
end
activeidx(activeidx==0) = [];
restartflg = false;

if ~isempty(activeidx)

   if ~restartflg
      for i = 1:length(activeidx)
         cubeidx = AData(activeidx(i)).cubeidx;
         UPPER = cubeidx.UPPER;
         SCALINGS = cubeidx.SCALINGS;
         M = AData(activeidx(i)).M;
         bidx = AData(activeidx(i)).bidx;
         thiscube = AData(activeidx(i)).cubelist(1,:);
         if isinf(thiscube(UPPER))
            [upperL,~,~,~,~,~,dgbL] = uball(M,bidx);
         else
            [upperL,~,~,~,~,~,dgbL] = uball(M,bidx,thiscube(SCALINGS),1);
         end
         if ~isinf(upperL)
            thiscube(SCALINGS) = dgbL(:)';
            thiscube(UPPER) = upperL;
         end
         AData(activeidx(i)).cubelist(1,:) = thiscube;
         AData(activeidx(i)).ScaledUb = upperL;
      end
   end

   % Update bounds
   [ActLb,LbIdx] = max( [AData.ScaledLb]./AllIOScalings );
   ActUb = max( [AData.ScaledUb]./AllIOScalings );

   cnt = 0;
   reasonstostop = [all( [AData.ScaledUb] <= AllIOScalings*GTH.A + GTH.M*AllScalednomgain); ...
      any( [AData.ScaledLb] >= AllIOScalings*BTH.A + BTH.M*AllScalednomgain); ...
      ActUb-ActLb<ABSTOL; ...
      ActUb-ActLb<RELTOL*ActUb; ...
      etime(clock,starttime)>MAXTIME; ...
      cnt >= MAXBRANCHES; ...
      strcmpi(LBONLY,'On')];

   while all(reasonstostop==0) && ~isempty(activeidx) 
      % Find offending (active) element
      [~,idx] = max( [AData(activeidx).ScaledUb]./AllIOScalings(activeidx) );
      idx = activeidx(idx);
      % Refine bounds
      AData(idx) = refineSP(AData(idx),BTH,NTIMES,MAXCNT);

      % Update bounds
      [ActLb,LbIdx] = max( [AData.ScaledLb]./AllIOScalings );
      ActUb = max( [AData.ScaledUb]./AllIOScalings );

      % Clean up: go through each element, and delete cubes with upper
      %   bounds that are lower than the current best LOWER bound
      for ii=1:length(activeidx)
         i = activeidx(ii);
         ACTIVE = AData(i).cubeidx.ACTIVE;
         UPPER = AData(i).cubeidx.UPPER;
         useless = AData(i).cubelist(:,ACTIVE)==1 & AData(i).cubelist(:,UPPER)<AllIOScalings(i)*ActLb;
         % deletecube (cube j in element i)
         AData(i).cubelist(useless,ACTIVE) = 0;
         % delete element
         if all(AData(i).cubelist(:,ACTIVE)==0)
            activeidx(ii) = 0;
         end
      end
      activeidx(activeidx==0) = [];

      cnt = cnt+1;
      reasonstostop = [all( [AData.ScaledUb] <= AllIOScalings*GTH.A + GTH.M*AllScalednomgain); ...
         any( [AData.ScaledLb] >= AllIOScalings*BTH.A + BTH.M*AllScalednomgain); ...
         ActUb-ActLb<ABSTOL; ...
         ActUb-ActLb<RELTOL*ActUb; ...
         cnt >= MAXBRANCHES; ...
         etime(clock,starttime)>MAXTIME];      
   end
end
   
% Store Output Data
CP = localGetCriticalPert(AData(LbIdx));
DataOut = struct('LowerBound',{ActLb},'UpperBound',{ActUb},...
            'CriticalIdx',{LbIdx},'CriticalAData',{AData(LbIdx)},...
            'AData',{AData},'CriticalPert',{CP});
% DataOut = struct('LowerBound',{ActLb},'UpperBound',{ActUb},...
%             'CriticalIdx',{LbIdx},'CriticalAData',{AData(LbIdx)},...
%             'CriticalArrayIdx',{[]},'CriticalFreqIdx',{[]},...
%             'CriticalFreq',{[]},'CriticalM',{[]},'CriticalB',{[]},...
%             'CriticalmuB',{[]},'WCU',{[]});


function PertData = localGetCriticalPert(AData)
% Packages worst-case perturbation data in cell-array format for
% use by getWorstCasePerturbation.
bidx = AData.bidx;
blk = bidx.simpleblk;
nblk = size(blk,1);
PertData = cell(nblk,1);

for k=1:bidx.full.num
   korig = bidx.full.origloc(k);
   PertData{korig} = {AData.AllWlvec{k},AData.AllWrvec{k}};
end
for k=1:bidx.sreal.num
   korig = bidx.sreal.origloc(k);
   PertData{korig} = AData.AllWsreal(k);
end
for k=1:bidx.repreal.num
   korig = bidx.repreal.origloc(k);
   PertData{korig} = AData.AllWrepreal(k);
end
for k=1:bidx.repcomp.num
   korig = bidx.repcomp.origloc(k);
   PertData{korig} = AData.AllWrepcomp(k);
end
