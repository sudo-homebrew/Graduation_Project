function Data = refineSP(Data,BTH,NTIMES,MAXCNT)
% Refines a single instance of a worst-case gain problem.  The problem is
% specified in the struct DATA, which has 18 fields.  REFINESP determines
% which parameter-cube should be divided, which parameter should be
% divided and where the division should occur.  Then the division takes
% place, and upper and lower bounds are calculated on both of the new
% "child" subcubes.  The information in DATA is updated, and returned.

%   Copyright 2010-2012 The MathWorks, Inc.
extdim = 50;
frac = 0.5;

cubelist = Data.cubelist;
M = Data.M;
IOScalings = Data.IOScalings;
Scalednomgain = Data.Scalednomgain;

bidx = Data.bidx;
cubeidx = Data.cubeidx;
LEFTCOR = cubeidx.LEFTCOR;
RIGHTCOR = cubeidx.RIGHTCOR;
LOWER = cubeidx.LOWER;
UPPER = cubeidx.UPPER;
ACTIVE = cubeidx.ACTIVE;
SCALINGS = cubeidx.SCALINGS;
SCALARS = cubeidx.SCALARS;
REPS = cubeidx.REPS;

% Reallocate cubelist if necessary
szcl = size(cubelist);
freespace = find(cubelist(:,ACTIVE)==0);
if isempty(freespace)
   freespace = szcl(1)+(1:extdim);
   cubelist = [cubelist; zeros(extdim,szcl(2)) ];
end

% Cube to split is: cubetosplit 
ac = find(cubelist(:,ACTIVE)==1);
[~,idx]= max( cubelist(ac,UPPER) );
cubetosplit = ac(idx(1));

% Split cube, and duplicate into two cubes
cubej = cubelist(cubetosplit,:);
[~,dimidx] = max(cubej(RIGHTCOR)-cubej(LEFTCOR));
k = dimidx(1);  % coordinate with longest side-length
% It is possible that additional "rough" calculation to make a more reasoned
% choice of dimension-to-split would be worthwhile.  There is no clear
% (other than a convergence proof) that the longest side is the best to
% split.  Moreover, the split location is also worth investigating.  For
% now, these are acceptable choices.  See Feeley thesis for how he
% addresses these choices in DataCollaboration.
left = cubej(LEFTCOR(k));
right = cubej(RIGHTCOR(k));
mid = left + frac*(right-left);
cubejleft = cubej;
cubejright = cubej;
cubejleft(RIGHTCOR(k)) = mid;
cubejright(LEFTCOR(k)) = mid;

% Recenter and normalize
McubeL = rcnrs(M,bidx,cubejleft(LEFTCOR),cubejleft(RIGHTCOR));
McubeR = rcnrs(M,bidx,cubejright(LEFTCOR),cubejright(RIGHTCOR));

% Call lower bounds
% Implicitly, we have [-1 , 1] assumed in the LOWERBOUND_CALC
MXGAIN = BTH.M*Scalednomgain + IOScalings*BTH.A;
RNG = RandStream('mt19937ar');
[lowerL,baddeltaLsreal,baddeltaLrepreal,...
  baddeltaLlvec,baddeltaLrvec,baddeltaLrepcomp,~] = ...
  wclowc(McubeL,bidx,NTIMES,MAXCNT,0,MXGAIN,RNG);
[lowerR,baddeltaRsreal,baddeltaRrepreal,...
  baddeltaRlvec,baddeltaRrvec,baddeltaRrepcomp,~] = ...
  wclowc(McubeR,bidx,NTIMES,MAXCNT,0,MXGAIN,RNG);
   
% New lower bounds (hopefully at least one is larger, though not necessarily)
cubejleft(1,LOWER) = lowerL;
cubejright(1,LOWER) = lowerR;
        
% Call upper bounds, using subdivided cube's scalings.
% Implicitly, we have [-1 , 1] assumed in the UPPERBOUND_CALC.
% Certificates are not currently returned to user, but might be used at a
% later time.  Recall output args are [upperL,di,dbs,gzr,gzl,psdami,dgbL].
if isinf(cubej(UPPER))
   [upperL,~,~,~,~,~,dgbL] = uball(McubeL,bidx);
   [upperR,~,~,~,~,~,dgbR] = uball(McubeR,bidx);
else
   [upperL,~,~,~,~,~,dgbL] = uball(McubeL,bidx,cubej(SCALINGS),1);
   [upperR,~,~,~,~,~,dgbR] = uball(McubeR,bidx,cubej(SCALINGS),1);
end
if ~isinf(upperL)
   cubejleft(SCALINGS) = dgbL(:)';
   cubejleft(1,UPPER) = upperL;
end
if ~isinf(upperR)
   cubejright(SCALINGS) = dgbR(:)';
   cubejright(1,UPPER) = upperR;
end

% New upper bounds (hopefully both are smaller, algorithm may not yield
% this, but it usually will).  We should not accept numbers when they get
% worse - just use the old ones -- but for now we accept.  Convergence is
% still guaranteed.
cubejleft(1,UPPER) = upperL;
cubejright(1,UPPER) = upperR;

Wgain = Data.ScaledLb/IOScalings; 

% Check if either is worst, unnormalize if so.
if (lowerL/IOScalings>=Wgain) && (lowerL>=lowerR)
   offset = (cubejleft(RIGHTCOR)+cubejleft(LEFTCOR))/2;
   slope = (cubejleft(RIGHTCOR)-cubejleft(LEFTCOR))/2;
   Data.AllWsreal = offset(SCALARS) + (baddeltaLsreal(:).').*slope(SCALARS);
   Data.AllWrepreal = offset(REPS) + (baddeltaLrepreal(:).').*slope(REPS);
   Data.AllWlvec = baddeltaLlvec;
   Data.AllWrvec = baddeltaLrvec;
   Data.AllWrepcomp = baddeltaLrepcomp;
   Data.ScaledLb = lowerL;
elseif (lowerR/IOScalings>=Wgain) && (lowerR>=lowerL)
   offset = (cubejright(RIGHTCOR)+cubejright(LEFTCOR))/2;
   slope = (cubejright(RIGHTCOR)-cubejright(LEFTCOR))/2;
   Data.AllWsreal = offset(SCALARS) + (baddeltaRsreal(:).').*slope(SCALARS);
   Data.AllWrepreal = offset(REPS) + (baddeltaRrepreal(:).').*slope(REPS);
   Data.AllWlvec = baddeltaRlvec;
   Data.AllWrvec = baddeltaRrvec;
   Data.AllWrepcomp = baddeltaRrepcomp;
   Data.ScaledLb = lowerR;
elseif lowerR/IOScalings<Wgain && lowerL/IOScalings<Wgain
   %disp('Splitting did not help lower bound');
end

% All info is ready in new cubes, so put this into BBList.  We overwrite
%   the one we split, as well as add to the BBList at FREESPACE(1)        
Data.cubelist(cubetosplit,:) = cubejleft;
Data.cubelist(freespace(1),:) = cubejright;
Data.ScaledUb = max( Data.cubelist(Data.cubelist(:,ACTIVE)==1,UPPER) );
