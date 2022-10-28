function [S,RefBnd,PertBnd] = getCSensitivity(M,blkData,gamma,opt,UncBlkIdx,FixedBlkIdx)
%     blkData   Block structure (length nblk). May include performance blocks 
%               at bottom in RG, WCG, WCDM cases
%   UncBlkIdx   These are the block for which sensitivity is computed.
%                  RS        1:nblk
%                  RG, WCG   1:nblk-1
%                  WCDM      1:nblk-nL
% FixedBlkIdx   These are the "fixed" blocks in skew-mu problems (needed to 
%               compute mu upper bound).
%                  RS        []
%                  RG        nblk
%                  WCG       1:nblk-1
%                  WCDM      1:nblk-nL
%        gamma  Finite-difference amount, 0.25 -> 25% finite-diff in 
%               independent variable

%   Copyright 1986-2020 The MathWorks, Inc.

% Need to make a tolerances object, since DGUpperBoundFV requires it.  With
% regard to input arguments, only the "opt" arguent (associated with 'a' or
% 'G' for upperbound) is used in DGUpperBoundFV.   Other values, such as
% LMIopt and osborneConditionNumber are also used.
V = ssmussvTolerances(opt, 0, [1 1],[0,Inf], true);

nFreqs = size(M,3);
nUncBlk = numel(UncBlkIdx);

S = zeros(nUncBlk,1);
FDfactor = 1 + gamma;
sFDfactor = sqrt(FDfactor);
if nUncBlk==1 && numel(FixedBlkIdx)==0
   S = 1;
   RefBnd = [];
   PertBnd = [];
else
   ridxm = blkData.ridxm;
   cidxm = blkData.cidxm;
   UBRef = 0;
   for j=1:nFreqs
      ub = constantDGub(M(:,:,j),blkData,V);
      UBRef = max(UBRef, ub);
   end
   for i=1:nUncBlk
      PertUBnd = 0;
      k = UncBlkIdx(i);
      Mscl = M;
      Mscl(ridxm(k):ridxm(k+1)-1,:,:) = sFDfactor*Mscl(ridxm(k):ridxm(k+1)-1,:,:);
      Mscl(:,cidxm(k):cidxm(k+1)-1,:) = sFDfactor*Mscl(:,cidxm(k):cidxm(k+1)-1,:);
      for j=1:nFreqs
         ub = constantDGub(Mscl(:,:,j),blkData,V);
         PertUBnd = max(PertUBnd, ub);
      end
      % SM = 1/mu; RG = 1/mu; WCG = 1/mu, for appropriate skewed (FV) mu
      % Compute sensitivity of SM or RG or WCG
      S(i) = (PertUBnd-UBRef)/(UBRef*(FDfactor-1));
      % Consider muN, muP, uradN, uradP, smN, smP
      %   percentage change in urad: (uradP-uradN)/uradN
      %      but uradP = FDfactor*uradN
      %      so: perc_uradChange = (F-1)
      %   percentage change in sm: (smN-smP)/smN
      %      in terms of mu: (1/muN - 1/muP)/(1/muN)
      %   S_sm = (percentage SM change)/(percentage radius change)
      %        = (1 - muN/muP)/(F - 1)
      %        = (muP - muN)/(muP*(F-1))
      %   by contrast, S_mu = (muP-muN)/(muN*(F-1))
   end
end
S = round(100*max(S,0));
