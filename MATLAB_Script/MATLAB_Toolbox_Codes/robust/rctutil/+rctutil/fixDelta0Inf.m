function LBcert = fixDelta0Inf(a,b,c,d,Ts,blkData,LBcert,muOpt)
% If Peak is at 0 or INF, and muB has complex-blocks, and the values
% in LBcert.Delta are actually complex, then we need to:
%    * not continuous at 0 (or Inf), in which case the complex blocks in
%      muB can be eliminated, call MUSSV with 'fg' option and get a
%      purely real uncertainty; or
%    * continuous at 0 (or Inf), perturb frequency just a bit, and get a
%      new Delta, with nonzero, finite w
%
% XXX: still need to address the discrete-time version.  Also, it would be
% useful to have some frequency grid passed in here, so as to select
% "small" or "large" freqeuncies in context of the particular system.

%   Copyright 1986-2020 The MathWorks, Inc.
w = LBcert.w;
Delta = LBcert.Delta;
FVidx = blkData.FVidx;
blk = blkData.simpleblk;
varyBlk = blk;
varyBlk(FVidx.fixedBlkIdx,:)=[];

ImagTol = 1e-6;
LB = 1/norm(Delta(FVidx.VaryCols,FVidx.VaryRows));
Changed = true;

% Note: 
% 1) TOL must be consistent with tolerances in MKPERT
% 2) Assumes Ts=0 for now
tol = eps^(3/4);
DCFlag = (abs(w)<=tol);
InfFlag = (abs(w)>=1/tol);
if w>=0
   sgn = 1;
else
   sgn = -1;
end

if DCFlag || InfFlag
   if all(Delta(:)==0)
      % No pert was found, so this is FV-OK
      DeltaCorrected = Delta;
      wCorrected = w;
      LBCorrected = 0;
      Changed = false;
   elseif all(imag(Delta(:))==0) || norm(imag(Delta))<ImagTol*norm(real(Delta))
      % Pert found was essentially real, which is OK
      DeltaCorrected = real(Delta);
      wCorrected = w;
      LBCorrected = 1/norm(Delta(FVidx.VaryCols,FVidx.VaryRows));
   else
      % recalc MU to see if complex blocks don't matter 
      LBCorrected = NaN;
      if blkData.allreal.num>0 && any(varyBlk(:,1)<0)
         realColIdx = blkData.allreal.allcols;
         realRowIdx = blkData.allreal.allrows;
         WorstLB = coreLowerBound(a,b(:,realColIdx),c(realRowIdx, :),...
            d(realRowIdx, realColIdx),Ts,w,blkData.allreal.realidx,'',...
            'complexify');
         if WorstLB.LB>=LB
            DeltaCorrected = zeros(size(d));
            DeltaCorrected(realColIdx, realRowIdx) = WorstLB.Delta;
            wCorrected = WorstLB.w;
            LBCorrected = 1/norm(DeltaCorrected(FVidx.VaryCols,FVidx.VaryRows));
         end
      end
      if isnan(LBCorrected)
         if DCFlag
            % Perturb w from 0 or INF (1, -1)
            wNZstart = sgn*1e-4;  % take first nonzero, or some fraction of it
            wAdjustFactor = 0.5;
         else
            wNZstart = sgn*1e4;  % take last finite, or some multiple of it
            wAdjustFactor = 2;
         end
         LBtol = 0.999;
         go = true;
         wCorrected = wNZstart;
         cntMax = 10;
         cnt = 1;
         while go && cnt<cntMax
            WorstLB = coreLowerBound(a,b,c,d,Ts,wCorrected,blkData,muOpt,'complexify');
            % GoffProb = rctutil.freqresp(a,b,c,d,Ts,wCorrected);
            % [muBnds,mui] = mussv(GoffProb,blk,muOpt);
            if WorstLB.LB>=LBtol*LB % watch for Inf
               DeltaCorrected = WorstLB.Delta;
               LBCorrected = 1/norm(DeltaCorrected(FVidx.VaryCols,FVidx.VaryRows));
               go = false;
            else
               wCorrected = wAdjustFactor*wCorrected;
               cnt = cnt + 1;
            end
         end
         if go
            % None of the fixes worked.  Resort to answer from current wCorrected
            WorstLB = coreLowerBound(a,b,c,d,Ts,wCorrected,blkData,muOpt,'complexify');
            wCorrected = WorstLB.w;
            DeltaCorrected = WorstLB.Delta;
            LBCorrected = 1/norm(DeltaCorrected(FVidx.VaryCols,FVidx.VaryRows));
         end
      end
   end
else
   Changed = false;
   DeltaCorrected = Delta;
   wCorrected = w;
   LBCorrected = 1/norm(Delta(FVidx.VaryCols,FVidx.VaryRows));
end

if Changed
   LBcert.Delta = DeltaCorrected;
   LBcert.w = wCorrected;
   LBcert.LB = LBCorrected;
end
