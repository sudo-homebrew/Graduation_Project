function [SM,WCU,INFO] = robUnstable(muB,BND)
% Populates ROB* outputs for nominally unstable systems.

%   Copyright 2004-2015 The MathWorks, Inc.

BlkInfo = muB.BlkInfo;
nblk = numel(BlkInfo);
BN = {BlkInfo.BlockName};  BN = BN(:);
SM = struct(...
   'LowerBound',BND,...
   'UpperBound',BND,...
   'CriticalFrequency',NaN);
% Worst perturbation is zero (in normalized units)
WCU = cell(nblk,1);
for ct=1:nblk
   blk = BlkInfo(ct).BlockData;
   if isa(blk,'DynamicSystem')
      [ny,nu] = size(blk);
      WCU{ct} = ltipack.ssdata([],zeros(0,nu),zeros(ny,0),zeros(ny,nu),[],blk.Ts);
   else
      WCU{ct} = blk.NominalValue;
   end
end
WCU = cell2struct(WCU,BN,1);
INFO = struct(...
   'Model',[],...
   'Frequency',NaN,...
   'Bounds',[BND BND],...
   'WorstPerturbation',WCU,...
   'Sensitivity',cell2struct(num2cell(NaN(size(BN))),BN,1));