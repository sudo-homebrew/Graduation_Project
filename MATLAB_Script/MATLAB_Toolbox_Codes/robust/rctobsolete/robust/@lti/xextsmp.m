function smp = xextsmp(sys,fullstatescale)
%XEXTSMP Compute stable, minimum phase LTI system.
% SMP = XEXTSMP(SYS)
%
%XEXTSMP computes stable, minimum phase LTI system, SMP,
%from the original LTI system, SYS, such that
%
%      (SMP')*SMP = (SYS')*SYS
%
%SYS should have full column-rank at all s=jw,
%and should have no poles on s=jw.
%
%See also: DKSYN, FITMAGFRD

%   Copyright 1991-2010 The MathWorks, Inc.
if nargin == 0
   disp('usage: smp = xextsmp(sys)');
   return
end
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
if nargin == 1
   fullstatescale = 1;
end
if isa(sys,'double') || isstatic(sys) %constant D term only
   smp = sys;
elseif isa(sys,'ss')
   szs = size(sys);
   mrows = szs(1);
   mcols = szs(2);
   ns = length(get(sys,'StateName'));
   [a,b,c,d] = ssdata(sys);
   if rank(d)~=mcols
      error('System should have Full Column Rank at s=infty')
   end
   aevl = abs(eig(a));
   sclalpha = sqrt(min(aevl)*max(aevl));
   a = a/sclalpha;
   b = b/sclalpha;
   normb = norm(b,'fro');
   normc = norm(c,'fro');
   m = [a b;c d];
   if normb>0 && normc>0
      if fullstatescale==0
         blk = [ones(ns,2);mcols mrows];
      else
         blk = [ns 0;mcols mrows];
      end
      [~,muinfo] = mussv(m,blk,'fsU');
      [dl,dr] = mussvunwrap(muinfo);
      % Andy: sept 5, 05, We should check the condition number here
%       if rcond(dl(1:ns,1:ns)) < 100*eps
%          warning('High condition-number state transformation in XEXTSMP');
%       end
      sysx = dl*m/dr;
      a = sysx(1:ns,1:ns);
      b = sysx(1:ns,ns+1:end);
      c = sysx(ns+1:end,1:ns);
      d = sysx(ns+1:end,ns+1:end);
   end
   if isequal(szs,[1 1])
      [~,Mb] = balance(m,'noperm');
      a = Mb(1:end-1,1:end-1);
      b = Mb(1:end-1,end);
      c = Mb(end,1:end-1);
      d = Mb(end,end);
   end
   aa = [-a' -c'*c; zeros(ns,ns) a];
   cc = [b' d'*c];
   bb = [-c'*d ; b];
   dd = d'*d;
   
   % Andy Mod's
   nx = size(a,1);
   [Vs,SigS] = LOCALgetseisp(a);
   ns = size(Vs,2);
   [Vu,SigU] = LOCALgetseisp(a,'rhp');
   nu = size(Vu,2);
   if ns+nu==nx
      W = inv([Vs Vu]');
      Ws = W(:,1:ns);
      Wu = W(:,ns+1:end);
      H = lyap(SigS',(c*Vs)'*(c*Vs));
      % BigS = [Wu Ws*H;zeros(nx,nu) Vs];
      [Q,~] = qr([Wu Ws*H;zeros(nx,nu) Vs]);
      BigS = Q(:,1:nx);
      K = lyap(SigU',(c*Vu)'*(c*Vu));
      % BigU = [Ws Wu*K;zeros(nx,ns) Vu];
      [Q,~] = qr([Ws Wu*K;zeros(nx,ns) Vu]);
      BigU = Q(:,1:nx);
      tran = [BigS BigU];
      newa = tran\aa*tran;
      a1 = newa(1:nx,1:nx);
      ct = cc*tran;
      c1 = ct(:,1:nx);
      tib = tran\bb;
      b1 = tib(1:nx,:);
      x = icare(a1,b1,0,dd,c1');
      if ~(size(x,1)==size(a1,1) && all(isfinite(x(:))))
         %                   disp('Riccati did not work,... trying to fix.')
         if mcols==1 && mrows==1
            pp = pole(sys);
            zz = zero(sys);
            if any(real(pp)==0) || any(real(zz)==0)
               %disp('Warning: Decomposition fails');
               smp = [];
            else
               ppr = find(real(pp)>0);
               pp(ppr) = -pp(ppr);
               zzr = find(real(zz)>0);
               zz(zzr) = -zz(zzr);
               smp = ss(tf(poly(zz),poly(pp)));
               omega = logspace(-4,4,9);
               fac = frd(sys,omega)/frd(smp,omega);
               rd = get(fac,'ResponseData');
               fac = mean(real(rd(:)));
               smp = fac*smp;
            end
         else
            %disp('Warning: Decomposition fails');
            smp = [];
         end
      else
         ddsm = sqrtm(dd);
         cc1 = ddsm\(c1+b1'*x);
         smp = ssbal(ss(sclalpha*a1,sclalpha*b1,cc1,ddsm));
      end
   else
      error('Subspace did not split properly')
   end
else
   error('Input variable should be a DOUBLE or SS object')
end

function [basis,arep] = LOCALgetseisp(a,flg)
if nargin==1
   flg = 'lhp';
end
N = size(a,1);
[u,t] = schur(a);
[us,ts] = ordschur(u,t,flg);
loc = 1;
go = 1;
while go && loc<=N
   if loc<N && abs(ts(loc+1,loc))>0
      tmp = ts([loc loc+1],[loc loc+1]);
      evl = eig(tmp);
      num = 2;
   else
      evl = ts(loc,loc);
      num = 1;
   end
   switch flg
      case 'lhp'
         if all(real(evl)<0)
            loc = loc + num;
         else
            go = 0;
         end
      case 'rhp'
         if all(real(evl)>0)
            loc = loc + num;
         else
            go = 0;
         end
      case 'udi';
         if all(abs(evl)<1)
            loc = loc + num;
         else
            go = 0;
         end
      case 'udo'
         if all(abs(evl)>1)
            loc = loc + num;
         else
            go = 0;
         end
   end
end
n = loc - 1;
basis = us(:,1:n);
arep = ts(1:n,1:n);
