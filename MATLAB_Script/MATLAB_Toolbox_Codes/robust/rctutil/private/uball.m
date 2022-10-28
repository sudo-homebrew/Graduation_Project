function [beta,di,dbs,gzr,gzl,psdami,xopt,ftdidx] = uball(m,idx,xinit,chkfeas,ex5)
%   This does upper bound on worst-case performance, in terms
%   of max(svd(LFT)), for an upper loop closure of an LFT,

%   Copyright 2003-2011 The MathWorks, Inc.

% July 15, 2002.  All occurances of DAUG replaced with BLKDIAG.

if nargin==0
   disp('[beta,di,dbs,gzr,gzl,psdami,xopt] = uball(m,idx,xinit,chkfeas)');
   return
end

if nargin==2
   xinit = [];
end
compute = 1;
if nargin==5
   compute = 0;
end

szm = size(m);
ne = szm(1) - idx.rdimm;
nd = szm(2) - idx.cdimm;
IcolM = eye(szm(2));

setlmis([]);
nvar = 0;

drealidx = [];
dimagidx = [];
grealidx = [];
gimagidx = [];
drealselect = [];
dimagselect = [];
grealselect = [];
gimagselect = [];
varcnt = 0;

% [varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]

% repeated FULL blocks
rpatRM = [];
ipatRM = [];
rpatCM = [];
ipatCM = [];
repfullrowM = [];
DRrepfull = zeros(idx.repfull.num,1);
DIrepfull = zeros(idx.repfull.num,1);
Erepfull = [];
for i=1:idx.repfull.num
   oloc = idx.repfull.origloc(i);
   rowM = idx.ridxm(oloc);
   sdim = idx.repfull.repeated(i);
   [DRrepfull(i),nvar,sxR] = lmivar(1,[idx.repfull.repeated(i) 1]);
   drealidx = [drealidx (varcnt+1):(varcnt+sdim*(sdim+1)/2)];
   drealselect = [drealselect;ksmidx(szm(1),rowM,rowM,sdim,idx.repfull.dim(i,2))];
   varcnt = varcnt + sdim*(sdim+1)/2;
   rpatRM = blkdiag(rpatRM,kron(sxR,eye(idx.repfull.dim(i,2))));
   rpatCM = blkdiag(rpatCM,kron(sxR,eye(idx.repfull.dim(i,1))));
   [DIrepfull(i),nvar,sxI] = lmivar(3,skewdec(idx.repfull.repeated(i),nvar));
   dimagidx = [dimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
   dimagselect = [dimagselect;kssmidx(szm(1),rowM,rowM,sdim,idx.repfull.dim(i,2))];
   varcnt = varcnt + sdim*(sdim-1)/2;
   ipatRM = blkdiag(ipatRM,kron(sxI,eye(idx.repfull.dim(i,2))));
   ipatCM = blkdiag(ipatCM,kron(sxI,eye(idx.repfull.dim(i,1))));
   repfullrowM = [repfullrowM idx.repfull.rows{i}];
   Erepfull = [Erepfull;IcolM(idx.repfull.cols{i},:)];
end
if idx.repfull.num>0
   [DRrepfull_RM] = lmivar(3,rpatRM);
   [DIrepfull_RM] = lmivar(3,ipatRM);
   [DRrepfull_CM] = lmivar(3,rpatCM);
   [DIrepfull_CM] = lmivar(3,ipatCM);
end
nvarrepfull = nvar;
%[varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]

% FULL blocks
rpatRM = [];
ipatRM = [];
rpatCM = [];
ipatCM = [];
fullrowM = [];
DRfull = zeros(idx.full.num,1);
Efull = [];
for i=1:idx.full.num
   oloc = idx.full.origloc(i);
   rowM = idx.ridxm(oloc);
   [DRfull(i),nvar,sxR] = lmivar(1,[1 1]);
   drealidx = [drealidx varcnt+1];
   drealselect = [drealselect;smidx(szm(1),rowM,rowM,1)];
   varcnt = varcnt + 1;
   rpatRM = blkdiag(rpatRM,sxR*eye(idx.full.dim(i,2)));
   rpatCM = blkdiag(rpatCM,sxR*eye(idx.full.dim(i,1)));
   fullrowM = [fullrowM idx.full.rows{i}];
   Efull = [Efull;IcolM(idx.full.cols{i},:)];
end
if idx.full.num>0
   [DRfull_RM] = lmivar(3,rpatRM);
   [DRfull_CM] = lmivar(3,rpatCM);
end
nvarfull = nvar;
%[varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]

% REAL, scalar
srealrowM = [];
Esreal = [];
for i=1:idx.sreal.num
   oloc = idx.sreal.origloc(i);
   rowM = idx.ridxm(oloc);
   colM = idx.cidxm(oloc);
   drealselect = [drealselect;smidx(szm(1),rowM,rowM,1)];
   grealselect = [grealselect;smidx(szm(1),rowM,colM,1)];
   srealrowM = [srealrowM idx.sreal.rows{i}];
   Esreal = [Esreal;IcolM(idx.sreal.cols{i},:)];
end
if idx.sreal.num>0
   [Dsreal,nvar,sxD] = lmivar(1,ones(idx.sreal.num,2));
   drealidx = [drealidx (varcnt+1):(varcnt+idx.sreal.num)];
   varcnt = varcnt + idx.sreal.num;
   [Gsreal,nvar,sxG] = lmivar(1,ones(idx.sreal.num,2));
   grealidx = [grealidx (varcnt+1):(varcnt+idx.sreal.num)];
   varcnt = varcnt + idx.sreal.num;
end
nvarsreal = nvar;
%[varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]

% repeated complex (alot like repeated full)
ipatRCM = [];
repcomprowM = [];
DIrepcomp = zeros(idx.repcomp.num,1);
Erepcomp = [];
DRstruct = [idx.repcomp.repeated ones(idx.repcomp.num,1)];
if idx.repcomp.num>0
   [DRrepcomp_RCM,nvar,sxR] = lmivar(1,DRstruct);
   for i=1:idx.repcomp.num
      sdim = DRstruct(i,1);
      oloc = idx.repcomp.origloc(i);
      rowM = idx.ridxm(oloc);
      drealselect = [drealselect;smidx(szm(1),rowM,rowM,sdim)];
      drealidx = [drealidx (varcnt+1):(varcnt+sdim*(sdim+1)/2)];
      varcnt = varcnt + sdim*(sdim+1)/2;
   end
end
for i=1:idx.repcomp.num
   oloc = idx.repcomp.origloc(i);
   rowM = idx.ridxm(oloc);
   sdim = DRstruct(i,1);
   [DIrepcomp(i),nvar,sxI] = lmivar(3,skewdec(sdim,nvar));
   dimagselect = [dimagselect;ssmidx(szm(1),rowM,rowM,sdim)];
   dimagidx = [dimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
   varcnt = varcnt + sdim*(sdim-1)/2;
   ipatRCM = blkdiag(ipatRCM,sxI);
   repcomprowM = [repcomprowM idx.repcomp.rows{i}];
   Erepcomp = [Erepcomp;IcolM(idx.repcomp.cols{i},:)];
end
if idx.repcomp.num>0
   [DIrepcomp_RCM,nvar,sxI] = lmivar(3,ipatRCM);
end
nvarrepcomp = nvar;
%[varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]

% repeated real
DIpatRCM = [];
GIpatRCM = [];
reprealrowM = [];
DIrepreal = zeros(idx.repreal.num,1);
GIrepreal = zeros(idx.repreal.num,1);
Erepreal = [];
DRstruct = [idx.repreal.repeated ones(idx.repreal.num,1)];
GRstruct = [idx.repreal.repeated ones(idx.repreal.num,1)];
if idx.repreal.num>0
   % all Dr first
   [DRrepreal_RCM,nvar,sxDR] = lmivar(1,DRstruct);
   for i=1:idx.repreal.num
      sdim = DRstruct(i,1);
      oloc = idx.repreal.origloc(i);
      rowM = idx.ridxm(oloc);
      drealselect = [drealselect;smidx(szm(1),rowM,rowM,sdim)];
      drealidx = [drealidx (varcnt+1):(varcnt+sdim*(sdim+1)/2)];
      varcnt = varcnt + sdim*(sdim+1)/2;
   end
   % next, Gr
   [GRrepreal_RCM,nvar,sxGR] = lmivar(1,GRstruct);
   for i=1:idx.repreal.num
      sdim = DRstruct(i,1);
      oloc = idx.repreal.origloc(i);
      rowM = idx.ridxm(oloc);
      colM = idx.cidxm(oloc);
      grealselect = [grealselect;smidx(szm(1),rowM,colM,sdim)];
      grealidx = [grealidx (varcnt+1):(varcnt+sdim*(sdim+1)/2)];
      varcnt = varcnt + sdim*(sdim+1)/2;
   end
end
%% DI, GI, DI, GI, ... , DI, GI
%for i=1:idx.repreal.num
%    sdim = DRstruct(i,1);
%    oloc = idx.repreal.origloc(i);
%    rowM = idx.ridxm(oloc);
%    colM = idx.cidxm(oloc);
%    [DIrepreal(i),nvar,sxDI] = lmivar(3,skewdec(sdim,nvar));
%    DIpatRCM = daug(DIpatRCM,sxDI);
%    [GIrepreal(i),nvar,sxGI] = lmivar(3,skewdec(sdim,nvar));
%    GIpatRCM = daug(GIpatRCM,sxGI);
%    reprealrowM = [reprealrowM idx.repreal.rows{i}];
%    Erepreal = [Erepreal;IcolM(idx.repreal.cols{i},:)];
%end
%if idx.repreal.num>0
%    [DIrepreal_RCM,nvar,sxDI] = lmivar(3,DIpatRCM);
%    % DI, DI, ... , DI
%    for i=1:idx.repreal.num
%        sdim = DRstruct(i,1);
%        oloc = idx.repreal.origloc(i);
%        rowM = idx.ridxm(oloc);
%        colM = idx.cidxm(oloc);
%        dimagselect = [dimagselect;ssmidx(szm(1),rowM,rowM,sdim)];
%        dimagidx = [dimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
%        varcnt = varcnt + sdim*(sdim-1)/2;
%    end
%    [GIrepreal_RCM,nvar,sxGI] = lmivar(3,GIpatRCM);
%    % GI, GI, ... , GI
%    for i=1:idx.repreal.num
%        sdim = DRstruct(i,1);
%        oloc = idx.repreal.origloc(i);
%        rowM = idx.ridxm(oloc);
%        colM = idx.cidxm(oloc);
%        gimagselect = [gimagselect;ssmidx(szm(1),rowM,rowM,sdim)];
%        gimagidx = [gimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
%        varcnt = varcnt + sdim*(sdim-1)/2;
%    end
%end
% DI, DI,
for i=1:idx.repreal.num
   sdim = DRstruct(i,1);
   [DIrepreal(i),nvar,sxDI] = lmivar(3,skewdec(sdim,nvar));
   DIpatRCM = blkdiag(DIpatRCM,sxDI);
   reprealrowM = [reprealrowM idx.repreal.rows{i}];
   Erepreal = [Erepreal;IcolM(idx.repreal.cols{i},:)];
end
% Gi GI
for i=1:idx.repreal.num
   sdim = DRstruct(i,1);
   [GIrepreal(i),nvar,sxGI] = lmivar(3,skewdec(sdim,nvar));
   GIpatRCM = blkdiag(GIpatRCM,sxGI);
end
if idx.repreal.num>0
   [DIrepreal_RCM,nvar,sxDI] = lmivar(3,DIpatRCM);
   % DI, DI, ... , DI
   for i=1:idx.repreal.num
      sdim = DRstruct(i,1);
      oloc = idx.repreal.origloc(i);
      rowM = idx.ridxm(oloc);
      colM = idx.cidxm(oloc);
      dimagselect = [dimagselect;ssmidx(szm(1),rowM,rowM,sdim)];
      dimagidx = [dimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
      varcnt = varcnt + sdim*(sdim-1)/2;
   end
   [GIrepreal_RCM,nvar,sxGI] = lmivar(3,GIpatRCM);
   % GI, GI, ... , GI
   for i=1:idx.repreal.num
      sdim = DRstruct(i,1);
      oloc = idx.repreal.origloc(i);
      rowM = idx.ridxm(oloc);
      colM = idx.cidxm(oloc);
      gimagselect = [gimagselect;ssmidx(szm(1),rowM,colM,sdim)];
      gimagidx = [gimagidx (varcnt+1):(varcnt+sdim*(sdim-1)/2)];
      varcnt = varcnt + sdim*(sdim-1)/2;
   end
end
nvarrepreal = nvar;
[lamvar,tnvar,sx] = lmivar(1,[2*nd 0]);

ftdidx.drealidx = drealidx;
ftdidx.dimagidx = dimagidx;
ftdidx.grealidx = grealidx;
ftdidx.gimagidx = gimagidx;
ftdidx.drealselect = drealselect;
ftdidx.dimagselect = dimagselect;
ftdidx.grealselect = grealselect;
ftdidx.gimagselect = gimagselect;

%[length(drealidx)   length(drealselect)]
%[length(dimagidx)    length(dimagselect)]
%[length(grealidx)    length(grealselect)]
%[length(gimagidx)     length(gimagselect)]
%[varcnt length(drealidx)+length(dimagidx)+length(grealidx)+length(gimagidx)]
%all = sort([drealidx dimagidx grealidx gimagidx]);
%[min(diff(all)) max(diff(all)) min(all) max(all) varcnt]

if compute==0
   beta = ftdidx;
   di = varcnt;
else
   lmicnt = 1; % start with actual LMI, add bounds
   % constant term  (Lhs)
   mbot = m(idx.rdimm+1:szm(1),:);
   cterm = mbot'*mbot;
   lmiterm([1 1 1 0],[real(cterm) -imag(cterm);imag(cterm) real(cterm)]);
   if idx.repfull.num>0
      mi = m(repfullrowM,:);
      % LHS, M' D M
      A = [real(mi)';-imag(mi)'];
      lmiterm([1 1 1 DRrepfull_RM],A,A');
      A = [imag(mi)';real(mi)'];
      lmiterm([1 1 1 DRrepfull_RM],A,A');
      B = [real(mi) -imag(mi)];
      lmiterm([1 1 1 DIrepfull_RM],A,B,'s');
      % RHS, D term
      Ez = 0*Erepfull;
      A = [Erepfull';Ez'];
      lmiterm([-1 1 1 DRrepfull_CM],A,A');
      A = [Ez';Erepfull'];
      lmiterm([-1 1 1 DRrepfull_CM],A,A');
      B = [Erepfull Ez];
      lmiterm([-1 1 1 DIrepfull_CM],A,B,'s');
      % Bounds (could do this without kronekar...)
      lmicnt = lmicnt + 1;
      lmiterm([-(lmicnt) 1 1 DRrepfull_CM],1,1);
      lmiterm([-(lmicnt) 2 2 DRrepfull_CM],1,1);
      lmiterm([-(lmicnt) 1 2 DIrepfull_CM],-1,1);
   end
   if idx.full.num>0
      mi = m(fullrowM,:);
      % LHS, M' D M
      A = [real(mi)';-imag(mi)'];
      lmiterm([1 1 1 DRfull_RM],A,A');
      A = [imag(mi)';real(mi)'];
      lmiterm([1 1 1 DRfull_RM],A,A');
      % RHS, D term
      Ez = 0*Efull;
      A = [Efull';Ez'];
      lmiterm([-1 1 1 DRfull_CM],A,A');
      A = [Ez';Efull'];
      lmiterm([-1 1 1 DRfull_CM],A,A');
      % Bounds (could do this without kronekar...)
      lmicnt = lmicnt + 1;
      lmiterm([-(lmicnt) 1 1 DRfull_CM],1,1);
   end
   if idx.sreal.num>0
      mi = m(srealrowM,:);
      % LHS, M' D M
      A = [real(mi)';-imag(mi)'];
      lmiterm([1 1 1 Dsreal],A,A');
      A = [imag(mi)';real(mi)'];
      lmiterm([1 1 1 Dsreal],A,A');
      % D (rhs)
      Ez = 0*Esreal;
      lmiterm([-1 1 1 Dsreal],[Esreal';Ez'],[Esreal Ez]);
      lmiterm([-1 1 1 Dsreal],[Ez';Esreal'],[Ez Esreal]);
      % j(M' G E - E' G M)
      A = [imag(mi)';real(mi)'];
      B = [Esreal Ez];
      lmiterm([-1 1 1 Gsreal],A,B,'s');
      A = [-real(mi)';imag(mi)'];
      B = [Ez Esreal];
      lmiterm([-1 1 1 Gsreal],A,B,'s');
      % Bounds
      lmicnt = lmicnt + 1;
      lmiterm([-(lmicnt) 1 1 Dsreal],1,1);
   end
   if idx.repcomp.num>0
      mi = m(repcomprowM,:);
      % LHS, M' D M
      A = [real(mi)';-imag(mi)'];
      lmiterm([1 1 1 DRrepcomp_RCM],A,A');
      A = [imag(mi)';real(mi)'];
      lmiterm([1 1 1 DRrepcomp_RCM],A,A');
      B = [real(mi) -imag(mi)];
      lmiterm([1 1 1 DIrepcomp_RCM],A,B,'s');
      % RHS, D term
      Ez = 0*Erepcomp;
      A = [Erepcomp';Ez'];
      lmiterm([-1 1 1 DRrepcomp_RCM],A,A');
      A = [Ez';Erepcomp'];
      lmiterm([-1 1 1 DRrepcomp_RCM],A,A');
      B = [Erepcomp Ez];
      lmiterm([-1 1 1 DIrepcomp_RCM],A,B,'s');
      % Bounds
      lmicnt = lmicnt + 1;
      lmiterm([-(lmicnt) 1 1 DRrepcomp_RCM],1,1);
      lmiterm([-(lmicnt) 2 2 DRrepcomp_RCM],1,1);
      lmiterm([-(lmicnt) 1 2 DIrepcomp_RCM],-1,1);
   end
   if idx.repreal.num>0
      mi = m(reprealrowM,:);
      % LHS, M' D M
      A = [real(mi)';-imag(mi)'];
      lmiterm([1 1 1 DRrepreal_RCM],A,A');
      A = [imag(mi)';real(mi)'];
      lmiterm([1 1 1 DRrepreal_RCM],A,A');
      B = [real(mi) -imag(mi)];
      lmiterm([1 1 1 DIrepreal_RCM],A,B,'s');
      % RHS, D term
      Ez = 0*Erepreal;
      A = [Erepreal';Ez'];
      lmiterm([-1 1 1 DRrepreal_RCM],A,A');
      A = [Ez';Erepreal'];
      lmiterm([-1 1 1 DRrepreal_RCM],A,A');
      B = [Erepreal Ez];
      lmiterm([-1 1 1 DIrepreal_RCM],A,B,'s');
      % RHS j(M'GE - E'GM)
      A = [imag(mi)';real(mi)'];
      B = [Erepreal Ez];
      lmiterm([-1 1 1 GRrepreal_RCM],A,B,'s');
      A = [-real(mi)';imag(mi)'];
      B = [Ez Erepreal];
      lmiterm([-1 1 1 GRrepreal_RCM],A,B,'s');
      A = [-real(mi)';imag(mi)'];
      B = [Erepreal Ez];
      lmiterm([-1 1 1 GIrepreal_RCM],A,B,'s');
      A = [-imag(mi)';-real(mi)'];
      B = [Ez Erepreal];
      lmiterm([-1 1 1 GIrepreal_RCM],A,B,'s');
      % Bounds
      lmicnt = lmicnt + 1;
      lmiterm([-(lmicnt) 1 1 DRrepreal_RCM],1,1);
      lmiterm([-(lmicnt) 2 2 DRrepreal_RCM],1,1);
      lmiterm([-(lmicnt) 1 2 DIrepreal_RCM],-1,1);
   end
   % scalar term (RHS)
   Eq1 = zeros(nd,szm(2));
   Eq1(:,szm(2)-nd+1:szm(2)) = eye(nd);
   Eq1 = kron(eye(2),Eq1);
   lmiterm([-1 1 1 lamvar],Eq1',Eq1);
   
   c = zeros(tnvar,1);
   c(tnvar) = 1;
   lmisys = getlmis;
   
   if nargin==4
      copt = xinit(end);
      dbs = copt*eye(szm(2),szm(2));
      di = eye(szm(1),szm(1));
      gzr = zeros(szm(1),szm(2)); % row M, col M
      gzl = zeros(szm(2),szm(1)); % col M, row M
      beta = sqrt(copt);
      
      if idx.repfull.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repfull.num
            rowp = [rowp idx.repfull.rows{i}];
            colp = [colp idx.repfull.cols{i}];
         end
         dmatR = dec2mat(lmisys,xinit,DRrepfull_RM)+sqrt(-1)*dec2mat(lmisys,xinit,DIrepfull_RM);
         dmatC = dec2mat(lmisys,xinit,DRrepfull_CM)+sqrt(-1)*dec2mat(lmisys,xinit,DIrepfull_CM);
         dbs(colp,colp) = dmatC;
         di(rowp,rowp) = dmatR;
      end
      if idx.full.num>0
         rowp = [];
         colp = [];
         for i=1:idx.full.num
            rowp = [rowp idx.full.rows{i}];
            colp = [colp idx.full.cols{i}];
         end
         dmatR = dec2mat(lmisys,xinit,DRfull_RM);
         dmatC = dec2mat(lmisys,xinit,DRfull_CM);
         dbs(colp,colp) = dmatC;
         di(rowp,rowp) = dmatR;
      end
      if idx.sreal.num>0
         rowp = [];
         colp = [];
         for i=1:idx.sreal.num
            rowp = [rowp idx.sreal.rows{i}];
            colp = [colp idx.sreal.cols{i}];
         end
         dmat = dec2mat(lmisys,xinit,Dsreal);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
         gmat = dec2mat(lmisys,xinit,Gsreal);
         gzr(rowp,colp) = gmat;
         gzl(colp,rowp) = gmat;
      end
      if idx.repcomp.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repcomp.num
            rowp = [rowp idx.repcomp.rows{i}];
            colp = [colp idx.repcomp.cols{i}];
         end
         dmat = dec2mat(lmisys,xinit,DRrepcomp_RCM)+sqrt(-1)*dec2mat(lmisys,xinit,DIrepcomp_RCM);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
      end
      if idx.repreal.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repreal.num
            rowp = [rowp idx.repreal.rows{i}];
            colp = [colp idx.repreal.cols{i}];
         end
         dmat = dec2mat(lmisys,xinit,DRrepreal_RCM)+sqrt(-1)*dec2mat(lmisys,xinit,DIrepreal_RCM);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
         gmat = dec2mat(lmisys,xinit,GRrepreal_RCM)+sqrt(-1)*dec2mat(lmisys,xinit,GIrepreal_RCM);
         gzr(rowp,colp) = gmat;
         gzl(colp,rowp) = gmat;
      end
      lhs1 = m'*di*m;
      rhs1 = dbs+sqrt(-1)*(m'*gzr-gzl*m);
      tmp = evallmi(lmisys,xinit);
%       for i=1:lmicnt
%          [lhs,rhs] = showlmi(tmp,i);
%          % disp(['Each LMIs  ' int2str(i) '/' int2str(lmicnt)])
%          evs = eig(rhs-lhs);
%          if min(evs)<0
%             disp(['(' int2str(i) ') Initialpoint not actually feasible ' num2str(min(evs))]);
%             disp(['Max evl:' num2str(max(evs))]);
%          end
%       end
   end
   
   opts = [0;0;0;0;1];
   %opts = [0;0;0;0;0];
   [copt,xopt] = mincx(lmisys,c,opts,xinit);
   if isempty(xopt)
      %warning('Unable to find feasible point');
      beta = inf;
      dbs = [];
      di = [];
      gzr = [];
      gzl = [];
      psdami = [];
   else
      dbs = copt*eye(szm(2),szm(2));
      di = eye(szm(1),szm(1));
      gzr = zeros(szm(1),szm(2)); % row M, col M
      gzl = zeros(szm(2),szm(1)); % col M, row M
      beta = sqrt(copt);
      
      if idx.repfull.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repfull.num
            rowp = [rowp idx.repfull.rows{i}];
            colp = [colp idx.repfull.cols{i}];
         end
         dmatR = dec2mat(lmisys,xopt,DRrepfull_RM)+sqrt(-1)*dec2mat(lmisys,xopt,DIrepfull_RM);
         dmatC = dec2mat(lmisys,xopt,DRrepfull_CM)+sqrt(-1)*dec2mat(lmisys,xopt,DIrepfull_CM);
         dbs(colp,colp) = dmatC;
         di(rowp,rowp) = dmatR;
      end
      if idx.full.num>0
         rowp = [];
         colp = [];
         for i=1:idx.full.num
            rowp = [rowp idx.full.rows{i}];
            colp = [colp idx.full.cols{i}];
         end
         dmatR = dec2mat(lmisys,xopt,DRfull_RM);
         dmatC = dec2mat(lmisys,xopt,DRfull_CM);
         dbs(colp,colp) = dmatC;
         di(rowp,rowp) = dmatR;
      end
      if idx.sreal.num>0
         rowp = [];
         colp = [];
         for i=1:idx.sreal.num
            rowp = [rowp idx.sreal.rows{i}];
            colp = [colp idx.sreal.cols{i}];
         end
         dmat = dec2mat(lmisys,xopt,Dsreal);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
         gmat = dec2mat(lmisys,xopt,Gsreal);
         gzr(rowp,colp) = gmat;
         gzl(colp,rowp) = gmat;
      end
      if idx.repcomp.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repcomp.num
            rowp = [rowp idx.repcomp.rows{i}];
            colp = [colp idx.repcomp.cols{i}];
         end
         dmat = dec2mat(lmisys,xopt,DRrepcomp_RCM)+sqrt(-1)*dec2mat(lmisys,xopt,DIrepcomp_RCM);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
      end
      if idx.repreal.num>0
         rowp = [];
         colp = [];
         for i=1:idx.repreal.num
            rowp = [rowp idx.repreal.rows{i}];
            colp = [colp idx.repreal.cols{i}];
         end
         dmat = dec2mat(lmisys,xopt,DRrepreal_RCM)+sqrt(-1)*dec2mat(lmisys,xopt,DIrepreal_RCM);
         dbs(colp,colp) = dmat;
         di(rowp,rowp) = dmat;
         gmat = dec2mat(lmisys,xopt,GRrepreal_RCM)+sqrt(-1)*dec2mat(lmisys,xopt,GIrepreal_RCM);
         gzr(rowp,colp) = gmat;
         gzl(colp,rowp) = gmat;
      end
      psdami = -m'*di*m+dbs+sqrt(-1)*(m'*gzr-gzl*m);
   end
end

