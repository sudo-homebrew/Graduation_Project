function [blkData,mussvOpt] = ssmussvSetUp(mublkNby2,mussvOpt,fixList)
% Setup for SSMUSSV* computation. 

%   Copyright 1986-2019 The MathWorks, Inc.

% Use LMI-based UB computation for small to mid-size problems
nDV = rctutil.nDVmussv(mublkNby2);
nDVLimit = 60;
if ~any(mussvOpt=='f') && nDV<nDVLimit && ~any(mussvOpt=='G') 
   mussvOpt = [mussvOpt 'a'];
end

% Compile block data
blkData = rctutil.mkBlkData(mublkNby2,fixList);
if any(mussvOpt=='a')
   % DTOL set to prevent D scalings from becoming too ill-conditioned
   % (negatively impacts carving step).
   if strcmp(blkData.problemType,'wcgain')
      dtol = 1e-4; % REVISIT: needed?
   else
      dtol = 1e-3;
   end
   DGLMI = rctutil.DGLMIsys(blkData,fixList);
   DGLMI.dtol = dtol;
   DGLMI.LMIopt = [3e-3 0 1/dtol 0 1];
   blkData.allDGlmi = DGLMI;
end
[~,realfixBlkIdx] = intersect(blkData.allreal.origloc,fixList);
blkData.allreal.realidx = rctutil.mkBlkData(blkData.allreal.realblk, realfixBlkIdx); 
blkData.allcomp.compidx = rctutil.mkBlkData(blkData.allcomp.compblk, []);