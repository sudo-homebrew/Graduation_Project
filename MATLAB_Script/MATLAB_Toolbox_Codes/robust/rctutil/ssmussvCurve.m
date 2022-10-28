function [FinalBnds,UBcert,LBcert] = ssmussvCurve(ProblemType,a,b,c,d,Ts,...
   blkData,userMuOpt,Focus,fGrid)
%

%   Copyright 1986-2016 The MathWorks, Inc.

% Parse inputs
nin = nargin;
if nin<8
   userMuOpt = '';
end
if nin<9 || isempty(Focus)
   Focus = [0,pi/Ts];
end
if nin<10
   fGrid = [];
end
blk = blkData.simpleblk; % usual N-by-2 block description

% Pick the base grid if none specified
if isempty(fGrid)
   dynRange = rctutil.getDynamicRange(eig(a));
   fGrid = localPickGrid(dynRange,min(Focus,pi/Ts));
   if ~(isreal(a) && isreal(b) && isreal(c) && isreal(d))
      fGrid = sort([-fGrid , fGrid]);
   end
end
   
% Compute lower and upper bounds on base grid
nf = numel(fGrid);
M = fresp(ltipack.ssdata(a,b,c,d,[],Ts),fGrid);
if ~any(userMuOpt=='s')
   fprintf('Computing bounds... ')
end
[bnds,tinfo] = mussv(M,blk,userMuOpt,blkData.FVidx.fixedBlkIdx);
[pertM,~,VLmi] = mussvextract(tinfo);
bnds = permute(bnds,[3 2 1]);

% Store data
for ct=nf:-1:1
   w = fGrid(ct); % can be negative
   ctVLmi = struct('Dr',VLmi.Dr(:,:,ct),'Dc',VLmi.Dc(:,:,ct),...
      'Grc',VLmi.Grc(:,:,ct),'Gcr',VLmi.Gcr(:,:,ct));
   UBcert(ct,1) = struct('Interval',[w w],'gUB',bnds(ct,1),...
      'VLmi',ctVLmi,'ptUB',bnds(ct,1),'w',w,'Jump',false);
   LB = struct('w',w,'LB',bnds(ct,2),'Delta',pertM(:,:,ct));
   LBcert(ct,1) = rctutil.fixDelta0Inf(a,b,c,d,Ts,blkData,LB,userMuOpt);
end

% Compute peak
[maxUB,idx]= max(bnds(:,1));
[~,UBcert2,LBcert2] = ssmussvPeak(ProblemType,a,b,c,d,Ts,...
    blkData,userMuOpt,Focus,UBcert(idx));
 
% UBcert2 may just certify maxUB on a wide interval (then 
% UBcert2.ptUB << UBcert2.gUB = maxUB and gUB is a crude upper bound 
% at UBcert2.w). Ignore gUB unless higher than maxUB.
for ct=1:numel(UBcert2)
   if UBcert2(ct).gUB<=maxUB
      UBcert2(ct).gUB = UBcert2(ct).ptUB;
   end
end

% Build results
UBcert = cat(1,UBcert2,UBcert);
LBcert = cat(1,LBcert2,LBcert);
[~,is] = unique([LBcert.w],'sorted');
UBcert = UBcert(is);
LBcert = LBcert(is);
FinalBnds = [max([UBcert.gUB]), max([LBcert.LB])];


%------------------

function w = localPickGrid(dynRange,Focus)
% Build frequency grid

% Intersect dynamic range with Focus
dynRange = max(Focus(1),min(Focus(2),dynRange));

% Pick grid spanning dynamic range plus some
fc = sqrt(prod(dynRange));
fmin = min(dynRange(1)/10,fc/50);
fmax = max(10*dynRange(2),50*fc);
lfmin = floor(log10(fmin));
lfmax = ceil(log10(fmax));
w = logspace(lfmin,lfmax,max(40,5*(lfmax-lfmin)));

% Clip to Focus
w = w(w>Focus(1) & w<Focus(2));
w = [max(10^(lfmin-5),Focus(1)) w min(10^(lfmax+5),Focus(2))];
