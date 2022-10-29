function [mIC,bsel] = simplifyLFT(mIC,BlkStruct,LevelOverride,MaxLevel)
%SIMPLIFYLFT  Eliminates redundant blocks in static LFT model.
%
%   The LFT model is given as LFT(DELTA,M) where M is a double array with 2
%   or 3 dimensions. The block structure DELTA is described by BLKSTRUCT
%   (the blocks must be sorted by name beforehand). 
%
%   This function provides two override mechanisms:
%     1) If LEVELOVERRIDE>=0, this value overrides the AutoSimplify levels
%        specified by BLKSTRUCT.Simplify
%     2) When LEVELOVERRIDE<0, MAXLEVEL specify a maximum simplification 
%        level for all blocks (set MAXLEVEL=Inf to always use the AutoSimplify 
%        setting)
%
%   SIMPLIFYLFT returns:
%     * The reduced model M
%     * The index vector BSEL of remaining blocks.

%   Copyright 2003-2011 The MathWorks, Inc.

% Get number of occurrences and AutoSimplify levels
Occurrences = cat(1,BlkStruct.Occurrences);
AutoSimplify = cat(1,BlkStruct.Simplify);

% Main loop
nblk = numel(BlkStruct);
nocc = sum(Occurrences);
isKept = false(nocc,1);
if nocc>0
   % First pass: Only use "basic" or lower
   Levels = min(2,AutoSimplify);
   for ct=1:nblk
      [mIC,BlkStruct] = rctutil.simplifyBlock(mIC,BlkStruct,ct,Levels(ct));
   end
   % Second pass: Reduce by sweeping the block list until number of 
   % occurrences settles down
   nocc = sum([BlkStruct.Occurrences]);
   previous = Inf;
   if LevelOverride<0
      Levels = min(AutoSimplify,MaxLevel);
   else
      Levels = LevelOverride(ones(nblk,1));
   end
   while nocc>0 && nocc<previous
      for ct=1:nblk
         [mIC,BlkStruct] = rctutil.simplifyBlock(mIC,BlkStruct,ct,Levels(ct));
      end
      previous = nocc;
      nocc = sum([BlkStruct.Occurrences]);
   end
end

% Construct BSEL
idx = 0;
for ct=1:nblk
   isKept(idx+1:idx+BlkStruct(ct).Occurrences,:) = true;
   idx = idx + Occurrences(ct);
end
bsel = find(isKept);
