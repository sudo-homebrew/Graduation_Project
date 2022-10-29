function DGlmisys = DGLMIsys(index,fixList,FullDG)
% LMI setup for MU upper bound computation.

%   Copyright 2004-2019 The MathWorks, Inc.

% When there is a single+full+complex varying block, DrV and DcV are
% set to identity and we solve a MINCX instead of GEVP
WCGainFlag = strcmp(index.problemType,'wcgain');
MUSYN = strcmp(index.problemType,'musyn');
if nargin<3
   FullDG = true(1,2);
end

% M is nR-by-nC
nR = index.rdimm;
nC = index.cdimm;
blk = index.simpleblk;
varBlk = setdiff(1:size(blk,1),fixList); % indices of varying blocks
% Varying D-scale indices used in WCGain
Vridx = [];
Vcidx = [];

% Compressed scalings are 
%    Dx = Dx1 + j * Dx2 ,  Gx = Gx1 + j * Gx2
% with Dx1,Gx1 symmetric and Dx2,Gx2 skew-symmetric. The decision variables
% are the free entries of Dx1,Dx2,Gx1,Gx2.
%
% Dr,DcV,DcF,Gcr are derived from Dx,Gx to account for non-square or
% repeated blocks:
%    Dx = Dr(irx,irx) = DcF(icx,icx)+DcV(icx,icx)
%    Gx = Gcr(icx,irx)
% The real-valued LMI variable for Dr is
%    [Dr1 Dr2;-Dr2 Dr1],   Dr = Dr1+j*Dr2
% and similarly for DcV, DcF, Gcr.

% Create decision variable maps for each LMI variable
ndecvars=0;
ridxm = index.ridxm; % block row indices for M
cidxm = index.cidxm; % block col indices for M
Dr1 = zeros(nR);  Dr2 = Dr1;
DcF1 = zeros(nC);  DcF2 = DcF1;
DcV1 = DcF1;  DcV2 = DcF2;
Gcr1 = zeros(nC,nR);  Gcr2 = Gcr1;
RepeatScalar = (blk(:,2)==0);
nx = sum(abs(blk(RepeatScalar,1))) + sum(~RepeatScalar) - WCGainFlag;
Dx1 = zeros(nx);   Dx2 = Dx1;
Gx1 = Dx1;   Gx2 = Gx1;
ix = 0;  irx = [];  icx = [];  ifull = [];
for ct=1:size(blk,1)
   ridx = ridxm(ct):ridxm(ct+1)-1;
   cidx = cidxm(ct):cidxm(ct+1)-1;
   if WCGainFlag && ct==varBlk
      % DcV=1 in WCGain
      Vridx = ridx;
      Vcidx = cidx;
   elseif RepeatScalar(ct)
      % Repeated real or complex scalar, size m-by-m
      m = abs(blk(ct,1));
      irx = [irx , ridx]; %#ok<AGROW>
      icx = [icx , cidx]; %#ok<AGROW>
      % D scaling
      if FullDG(1)
         symblk  = symdec(m,ndecvars);
         ndecvars = ndecvars+m*(m+1)/2;
         skewblk  = skewdec(m,ndecvars);
         ndecvars = ndecvars+m*(m-1)/2;
         Dx1(ix+1:ix+m,ix+1:ix+m) = symblk;
         Dx2(ix+1:ix+m,ix+1:ix+m) = skewblk;
         Dr1(ridx,ridx) = symblk;
         Dr2(ridx,ridx) = skewblk;
         if any(ct==fixList)
            DcF1(cidx,cidx) = symblk;
            DcF2(cidx,cidx) = skewblk;
         else
            DcV1(cidx,cidx) = symblk;
            DcV2(cidx,cidx) = skewblk;
         end
      else
         % D restricted to diagonal
         diagblk =  diag(ndecvars+1:ndecvars+m);
         ndecvars = ndecvars+m;
         Dx1(ix+1:ix+m,ix+1:ix+m) = diagblk;
         Dr1(ridx,ridx) = diagblk;
         if any(ct==fixList)
            DcF1(cidx,cidx) = diagblk;
         else
            DcV1(cidx,cidx) = diagblk;
         end
      end
      % G scaling
      if blk(ct,1)<0 
         % repeated real scalar
         if FullDG(2)
            symblk  = symdec(m,ndecvars);
            ndecvars = ndecvars+m*(m+1)/2;
            skewblk  = skewdec(m,ndecvars);
            ndecvars = ndecvars+m*(m-1)/2;
            Gx1(ix+1:ix+m,ix+1:ix+m) = symblk;
            Gx2(ix+1:ix+m,ix+1:ix+m) = skewblk;
            Gcr1(cidx,ridx) = symblk;
            Gcr2(cidx,ridx) = skewblk;
         else
            % G restricted to diagonal
            diagblk =  diag(ndecvars+1:ndecvars+m);
            ndecvars = ndecvars+m;
            Gx1(ix+1:ix+m,ix+1:ix+m) = diagblk;
            Gcr1(cidx,ridx) = diagblk;
         end
      end
      if m>1
         ifull = [ifull , ix+1:ix+m]; %#ok<AGROW>
      end
      ix = ix+m;
   else
      % complex full block
      ndecvars = ndecvars+1;
      irx = [irx , ridx(1)]; %#ok<AGROW>
      icx = [icx , cidx(1)]; %#ok<AGROW>
      Dx1(ix+1,ix+1) = ndecvars;
      Dr1(ridx,ridx) = ndecvars*eye(blk(ct,2));
      if any(ct==fixList)
         DcF1(cidx,cidx) = ndecvars*eye(blk(ct,1));
      else
         DcV1(cidx,cidx) = ndecvars*eye(blk(ct,1));
      end
      ix = ix+1;
   end
end

% Build LMI
setlmis([]);

% Create compressed variables Dx and Gx
% Note: First to facilitate MAT2DEC-based initialization
Gx = [];
hasG = any(blk(:,1)<0);
cplxDG = any(Dx2,'all');
if cplxDG
   Dx = lmivar(3,[Dx1 Dx2; -Dx2 Dx1]);
   if hasG
      Gx = lmivar(3,[Gx1 Gx2;-Gx2 Gx1]);
   end
else
   % D,G diagonal
   Dx = lmivar(3,Dx1);
   if hasG
      Gx = lmivar(3,Gx1);
   end
end

% Create Dr, DcF, DcV
if isempty(fixList)
   DcF = [];
else
   DcF = lmivar(3,[DcF1 DcF2; -DcF2 DcF1]);
end
if WCGainFlag
   % No varying D-scale
   DcV = [];
else
   DcV = lmivar(3,[DcV1 DcV2; -DcV2 DcV1]);
end
Dr = lmivar(3,[Dr1 Dr2;-Dr2 Dr1]);

% Create Gcr
if hasG
   Gcr = lmivar(3,[Gcr1 Gcr2;-Gcr2 Gcr1]);
else
   Gcr = [];  
end

% Diagonal dominance constraints: Create variables for diagonal and 
% off-diagonal parts of Dx(ifull,ifull) and Gx(ifull,ifull), the
% portions of Dx,Gx associated with full D,G blocks (repeated scalars)
if MUSYN && FullDG(1) && ~isempty(ifull)
   % Note: cplxDG=true when D,G are not diagonal
   Dxf1 = Dx1(ifull,ifull);
   Dxf2 = Dx2(ifull,ifull);
   Dxf = [Dxf1 Dxf2; -Dxf2 Dxf1];
   aux = diag(diag(Dxf));
   Dxfd = lmivar(3,aux);      % diagonal part
   Dxfo = lmivar(3,Dxf-aux);  % off-diagonal part
else
   Dxfd = [];  Dxfo = [];
end

% Build summary
e100Mat = diag(diag(double(DcF1>0)));
DGlmisys = struct(...
   'lmisysInit',getlmis(),...
   'cplxDG',cplxDG,...   % True if D,G are complex valued
   'Dx',Dx,...
   'Gx',Gx,...
   'Dxfd',Dxfd,...
   'Dxfo',Dxfo,...
   'Dr',Dr,...
   'DcF',DcF,...
   'DcV',DcV,...
   'Gcr',Gcr,...
   'irx',irx,...   % Dx = Dr(irx,irx)
   'icx',icx,...   % Gx = Gcr(icx,irx)
   'numVARs',ndecvars,...
   'e100Mat',blkdiag(e100Mat,e100Mat), ...
   'Vridx',Vridx, ...
   'Vcidx',Vcidx, ...
   'alpha',5,...
   'dtol',[],...
   'LMIopt',[]);