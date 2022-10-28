function [ub,Dr,Dc,Gcr] = mufastub(M,index,bnds)
% function [ub,Dr,Dc,Gcr] = mufastub(M,index,bnds);
%
% 12/23/2005 PJS
% Based on deebal, geestep and rub code by Young/Newlin

% Copyright 2011-2014 The MathWorks, Inc.

% Get problem data
[Mr,Mc]=size(M);
realr = index.allreal.allrows;
realc = index.allreal.allcols;
%lreal = length(realr);
G_mask = index.masks.G_mask;

% Initialize bisection bounds
if nargin<3
    ub = max( [max(svd(M)), 10*eps] );
    lb = 0;
else
    ub = bnds(1);
    lb = bnds(2);
end
ubhigh = ub;
ublow = max(lb,1e-3);  % XXX Nov 21 2013: 10*eps -> 1e-3

%-------------------------------------------------------------------
% Choose G to cancel skew-Hermitian part of M in the balanced form
% Diagonalize this choice by wrapping orthog. transformation back
% into D-scales.
%-------------------------------------------------------------------
G = M(realr,realc).*G_mask;
G = (G - G')/2i; 
[gu,gdiag]=schur(G);  
gdiag = diag(gdiag);

dMd = M;
dMd(realr,:) = gu'*dMd(realr,:);
dMd(:,realc) = dMd(:,realc)*gu;

% Bisect in balanced form to find best (i.e. smallest)
% upper bound which satisfies
%         norm(Lterm*(Cterm/ub)*Rterm)<1
Lterm = ones(Mr,1);
Rterm = ones(Mc,1);
Cterm = dMd;
Cterm(realr,realc) = Cterm(realr,realc)-diag(1i*gdiag);
gdiag2 = gdiag.^2;
while (ubhigh-ublow)> 0.001*ublow  
    ubtry = (ublow+ubhigh)/2;
    gtry2 = gdiag2/(ubtry.^2);
    Lterm(realr) = (1+gtry2).^(-0.25);
    Rterm(realc) = Lterm(realr);
    if norm((Cterm/ubtry).*(Lterm*Rterm')) < 1        
        ubhigh=ubtry;
    else
        ublow =ubtry;
    end
end

% Store best g-scale obtained in balanced form
if ubhigh<ub
    ub = ubhigh;
    g	= gdiag/ub;
else
    g	= 0*gdiag;
end

%-------------------------------------------------------------------
% Do a single gradient step on g in balanced form.  
%-------------------------------------------------------------------
Lterm(realr) = (1+g.^2).^(-0.25);
Rterm(realc) = Lterm(realr);
Cterm = dMd/ub;
Cterm(realr,realc) = Cterm(realr,realc)-diag(1i*g);
[uu,ss,vv]=svd(Cterm.*(Lterm*Rterm'));
smax	= max([max(diag(ss)) 10*eps]);
sidx	= find((diag(ss)/smax)>0.95); % find s.v. clustered near max
if isempty(sidx)
    % This can happen if ss=0, e.g. for mussv(5*i,[-1 0])
    sidx = 1;
end    
lidx = length(sidx); 
uu    = uu(realr,sidx);
ss   = ss(sidx,sidx);
vv   = vv(realc,sidx);

% Compute ds_dg, the gradient of largest singular vals w.r.t. g
gtmp1 = repmat( g./(1+g.^2)/2 , [1 lidx]);
gtmp2 = repmat( (1+g.^2).^(-0.5) , [1 lidx]);
ds_dg = -real( ((abs(uu).^2 + abs(vv).^2).*gtmp1)*ss ...
    + 1i*conj(uu).*vv.*gtmp2 );

% Let g(t)=g0+t*gdir and compute the descent direction, gdir, to give a
% desired derivative of the largest singular values w.r.t to t, ds_dt.
ds_dt_des = -10*(diag(ss)-0.9);
gdir = pinv(ds_dg')*ds_dt_des;
ds_dt = ds_dg(:,1)'*gdir;

% Compute ds_dub, the negative of the gradient of largest 
% singular val w.r.t. ub. Then compute dub_dt, the gradient of the 
% upper bound w.r.t. t. Compute initial step, t0, to reduce upper
% bound by dub (based on first order Taylor series).
ds_dub = real( smax + 1i*uu(:,1)'*(gtmp2(:,1).*vv(:,1).*g) )/ub;
dub_dt = ds_dt/ds_dub;
dub = (lb - ub)/2;
t0 = dub/dub_dt;
t0 = min( max(t0,-1e6) , 1e6 ); % ensure t0 is bounded

% Take initial step and then backtrack, if necessary. 
gt = g+t0*gdir;  
Lterm(realr) = (1+gt.^2).^(-0.25);
Rterm(realc) = Lterm(realr);
Cterm = dMd/ub;
Cterm(realr,realc) = Cterm(realr,realc)-diag(1i*gt);
st=norm(Cterm.*(Lterm*Rterm'));
cnt = 0;
while st>smax && (cnt<10)
    t0 = t0/2;
    gt = g+t0*gdir;  
    Lterm(realr) = (1+gt.^2).^(-0.25);
    Rterm(realc) = Lterm(realr);
    Cterm = dMd/ub;
    Cterm(realr,realc) = Cterm(realr,realc)-diag(1i*gt);
    st=norm(Cterm.*(Lterm*Rterm'));
    cnt = cnt+1;
end

% Update g if smax is reduced.
if st<smax
    g = gt;
end

%-------------------------------------------------------------------
% Convert from Balanced to LMI form
%-------------------------------------------------------------------
gbal = (1+g.*g).^(-0.5);
Dtmp = gu*diag(gbal)*gu';

Dr = eye(Mr);
Dr(realr,realr) = Dtmp;

Dc = eye(Mc);
Dc(realc,realc) = Dtmp;

Gcr= zeros(Mc,Mr);
Gcr(realc,realr)=ub*(gu*diag(g.*gbal)*gu');

% FV certification
if ~isempty(index.FVidx.fixedBlkIdx)
   FixedCols = index.FVidx.FixedCols;
   VaryCols = index.FVidx.VaryCols;
   DcF = zeros(Mc);
   DcV = zeros(Mc);
   DcF(FixedCols,FixedCols) = Dc(FixedCols,FixedCols);
   DcV(VaryCols,VaryCols) = Dc(VaryCols,VaryCols);
   GcrM = Gcr*M;
   A = 1i*(GcrM - GcrM')+M'*Dr*M-DcF;
   B = DcV;
   evals = eig(A, B);
   finiteIdx = ~isinf(evals);
   ubsq = 1.0001*max(real(evals(finiteIdx)));
   if max(real(eig(A-ubsq*B)))<=0
      ub = sqrt( max(0,ubsq) );
   else
      ub = inf;
   end
else
   % Re-compute upper bound based on updated g-scale
   ubsq = max(real(eig( M'*Dr*M+1i*(Gcr*M-M'*Gcr') , Dc )));
   ub = sqrt( max(0,ubsq) );
end
