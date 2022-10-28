function [nmat,umat,umati] = gfactord(sys)
% Computes a stable, minimum phase SYSTEM
% matrix NMAT such that SYS = UMAT^(-1)*NMAT
% with UMAT inner and UMAT^(-1) = UMATI.
% Assumes SYS is a CONSTANT or SYSTEM matrix
% which is square and invertible

% P. M. Young, December, 1996.

%   Copyright 2009-2018 The MathWorks, Inc.
[a,b,c,d] = ssdata(sys);
mrows = size(sys,1);
ns = size(a,1);

% First of all we see if we need to stabilize the system
% and do it ONLY if required  (also only invert out the 
% minimal unstable part required)
[aord,trans,sizstab] = LOCALstaba(a);

if sizstab == ns
   umat = eye(mrows);  
   umati = eye(mrows);
else
   sizu = ns-sizstab;
   a = aord;  
   b = trans\b;  
   c = c*trans;
   a1 = a(1:sizstab,1:sizstab);  
   a2 = a(sizstab+1:ns,sizstab+1:ns);
   b1 = b(1:sizstab,:);  
   b2 = b(sizstab+1:ns,:);
   c1 = c(:,1:sizstab);  
   c2 = c(:,sizstab+1:ns);
   [x,k] = icare(a2',c2',0,-1);  % k = -c2*x
   if ~(size(x,1)==sizu && all(isfinite(x(:))))
      error('jw axis eigenvalues in Hamiltonian for D poles')
   end
   l = k';
   a = [a1 zeros(sizstab,sizu);-l*c1 a2-l*c2];
   b = [b1;b2-l*d];
   umat = ss(a2-l*c2,-l,c2,eye(mrows),sys.Ts);
   umati = ss(a2,-l,-c2,eye(mrows),sys.Ts);
end

% Now see if we need to make the system minimum phase 
% and do it ONLY if required - by essentially repeating
% the above on the inverse

atw = a-(b/d)*c;  
btw = b/d;
[aord,trans,sizstab] = LOCALstaba(atw);

if sizstab ~= ns
   sizu = ns-sizstab;
   a = (trans\a)*trans;  
   b = trans\b;  
   ch = c*trans;
   c1h = ch(:,1:sizstab);  
   c2h = ch(:,sizstab+1:ns);
   atw = aord;  
   btw = trans\btw;
   a2t = atw(sizstab+1:ns,sizstab+1:ns);  
   b2t = btw(sizstab+1:ns,:);
   [x,k] = icare(a2t,b2t,0,-1);  % k = -b2t'*x
   if ~(size(x,1)==sizu && all(isfinite(x(:))))
      error('jw axis eigenvalues in Hamiltonian for D poles')
   end
   c = [c1h c2h+k];
   numat = ss(a2t,b2t,k,eye(mrows),sys.Ts);
   numati = ss(a2t-b2t*k,b2t,-k,eye(mrows),sys.Ts);
   umat = numat*umat;  
   umati = umati*numati;
end 

if ns > 0
   nmat = ss(a,b,c,d,sys.Ts);
else
   nmat = d;
end

function [aord,trans,sizstab] = LOCALstaba(a)
%
% Returns a real block diagonal matrix aord, split
% into its stable and unstable subspaces, via
% the real transformation trans.  The stable subspace
% has dimension sizstab.  Assumes a is real and
% diagonalizable.

a = real(a);  tol = 1e-10;  siz = max(size(a));
[v,e] = eig(a);
[~,ind] = sort(real(diag(e)));
v = v(:,ind);   e = e(ind,ind);
sizstab = length(find(real(diag(e))<=0));

if (sizstab==0) || (sizstab==siz)
   aord = a;
   trans = eye(siz);
   return
end

aord = [];   trans = [];   skipblk = [];

for i=1:siz
   if isempty(skipblk)
     skipblkemp = 1;
   else
     if ~(any(skipblk==i))
      skipblkemp = 1;
     else
      skipblkemp = 0;
     end
   end
   if skipblkemp == 1
       if abs(imag(e(i,i))) <= tol
         aordblk = real(e(i,i));
         transblk = real(v(:,i));
         if norm(transblk) <= tol
            transblk = imag(v(:,i));
         end % if norm(
      else
         ind = find(abs(diag(e)-e(i,i)') <= tol);   
         skipblk = [skipblk ind(1)];
         lam1 = real(e(i,i));  lam2 = imag(e(i,i));
         x1 = real(v(:,i));   x2 = imag(v(:,i));
         aordblk = [lam1 lam2;-lam2 lam1];
         transblk = [x1 x2];
      end % if abs(
      aord = daug(aord,aordblk);
      trans = [trans transblk];
   end % if ~(any
end % for i

% check = max(size(trans));
%if check ~= siz
%   error('sorting error occured in staba')
%   return
%end


