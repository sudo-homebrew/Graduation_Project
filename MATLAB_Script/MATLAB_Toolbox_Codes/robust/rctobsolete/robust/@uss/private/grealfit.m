% function [sys,fit,axplt] = grealfit(vmagdata,dim,wt)
%
% GREALFIT fits a transfer function (magnitude and phase)
% to real data, VMAGDATA, with a supplied frequency
% domain weighting function, WT. VMAGDATA is a VARYING
% matrix giving the data and frequency points to be
% matched.  DIM is a 1 by 4 vector with dim= [hmax,htol,nmin,nmax].
% The output system, SYS, is a SISO system whose phase
% is always entirely real and, (modulo some normalizations),
% attempts to find the minimum H to satisfy:
%
%             (M - h/W) < g < (M + h/W)
%
% where g is the frequency response of SYS for frequencies omega.
% The degree of the numerator of SYS is 2*N, and the denominator
% is 2*(N+1), so that SYS is a strictly proper real rational function
% of s^2 (hence the phase is always real).  Here 0<=NMIN<=N<=NMAX is
% the minimum degree to meet the specificaton with H<=HMAX. If
% no such n exists then N=NMAX.  HTOL sets the tolerance to which the
% search is carried out, and FIT is the max weighted error of the
% final fit.

% The method used is to note that g is the ratio of polynomials in omega^2,
% g = p/q.  A scaled version of the  following linear program is then solved:
%
%  max epp, s.t.
%
% (epp/W)+(M-h/W)*(q/qbar) <= p/qbar <= (M+h/W)*(q/qbar)-epp/W
%
% (the coefs. of  p and q are named a and b in the code), and W is initially
% ones if not specified, and qbar is an estimate of q.  If epp>0 then a
% solution exists.  The dual LP is solved via modifications to LP and LINP
% called MFLINP and MFLP.  For sufficiently large (positive or negative)
% points only one of these inequalities is enforced, and these one sided
% inequalities are used when computing FIT.
%
% If NMIN or NMAX is > the number of data points, PTS, then they are set
% equal to PTS to avoid a degenerate problem.  If the input data is
% approximately zero then that is immediately returned as the fit.

% The code assumes that it is passed data in the form of g/w and wt*w,
% and tries to get the best fit to g, weighted by wt (the normalizations
% are carried out accordingly).  The final value of FIT is for this problem.

% P. M. Young, December, 1996.
% GJB modified 12 Sept 08

%   Copyright 2009 The MathWorks, Inc.

function [sys,fit,axplt] = grealfit(vmagdata,dim,Wt)

if nargin < 2
   disp('usage: [sys,fit,axplt] = grealfit(vmagdata,dim)')
   return
elseif any(size(dim) ~= [1 4])
   disp('usage: [sys,fit,axplt] = grealfit(vmagdata,dim)')
   return
end % if nargin

% Read in arguments and do some simple error checks

zerotol = 1e-2;   
bigtol = 1e50;  

% May want to change these tolerances ??
% If bigtol is >= gclip in MSFN then it effectively
% never enters into the problem

hmax=dim(1);  
htol=dim(2);  
nmin=dim(3);  
nmax=dim(4);
[rows,cols] = size(vmagdata);
if ~isa(vmagdata,'frd')
    error('VMAGDATA must be a FRD object')
end
magdata = vmagdata.ResponseData;
omega = vmagdata.Frequency;
pts = size(omega,1);

% Specify denominator order (in s^2) as nmin+1, and numerator 
% order as nmin, so true numerator order (in s) is 2*nmin, and true 
% denominator order is 2*(nmin+1).  If necessary reduce nmin and nmax 
% to avoid degeneracy in LP problem. 

nmin = nmin+1;	
nmax = nmax+1;

if nmax > pts
    nmax = pts;  
end
if nmin > pts
    nmin = pts;  
end

% Note we fit in s^2, so we square the frequencies, but not
% the magnitude data.  

om=omega.^2;	
mag=real(magdata);	

if nargin==2
   Wdata = ones(pts,1);  
   Winv = Wdata; 
else 
   Wt = abs(Wt);
   Wdata = Wt.ResponseData(:);
   Winv=Wdata.^(-1);
end % if nargin==2

omegas = omega;

% Keep mag as g/w (given) and define magw as g
% Also find points where g is too large (positive or negative)
% Form magtmp as a "clipped" version of g (later becomes g/w)
% and form axplt for plotting

magw   = omegas(:).*mag(:);
indpos = find(magw>=bigtol);
indneg = find(magw<=((-1)*bigtol));
lengthrm = length(indpos) + length(indneg);
magtmp = magw;   
magtmp(indpos) = bigtol*ones(length(indpos),1);
magtmp(indneg) = (-1)*bigtol*ones(length(indneg),1);
xplt = [floor(log10(min(omegas))) ceil(log10(max(omegas)))];
yplt = [min((1.1)*magtmp) max((1.1)*magtmp)];
axplt = [xplt yplt];
if lengthrm~=0
    hmax = bigtol*hmax;
    htol = htol*bigtol;
end

% Define average frequency and magnitude values.

n=nmin;
if om(1)==0
   omav=sqrt(om(2)*om(pts));
else 
   omav=sqrt(om(1)*om(pts));
end % if om(1)

% Define average magnitude, weighted by w, so that
% we get a correct average for g, assuming we
% have been given g/w - check this ??

magav = norm(magtmp,1)/pts;
if (magav<zerotol) && (lengthrm==0)
   sys = tf(0);  fit = max(abs(mag(:).*Wdata(:)));
   return
end

% Now magtmp becomes clipped version of g/w

magtmp=magtmp./omegas(:);

% Rescale frequency and magnitude for numerical reasons.

mag = mag/magav;  om=om/omav;	bold=ones(n+1,1);   
magtmp = magtmp/magav;   

% Initialize h-iteration variables.

h=hmax;  hupper=hmax;  hlower=0;
if htol>=0.5*hmax,  htol=0.49*hmax;  end,

% Initialize matrices for the LP with n=nmin.

foundn=0;  
M = (om*ones(1,n+1)).^(ones(pts,1)*[n:-1:0]);
inc = floor((pts-1)/(2*n+1));
startbasic = [2:2*inc:2*(n-1)*inc+2,pts+inc+2:2*inc:pts+2+(2*n+1)*inc];

while((hupper-hlower)>htol*max(hupper/hmax,1))

% Probably should zero protect qbar later ??
   
   qbar=polyval(bold,om);
   ophw=magtmp+h*Winv;  
   ophwinv=magtmp-h*Winv;
   P=(ophw*ones(1,n)).*M(:,2:n+1);
   R=(ophwinv*ones(1,n)).*M(:,2:n+1); 

%  May want to add some extra constraints here to allow
%  better fitting for large g's ??

   extraA = [];   lngthexa=0;  
   extraC = [];   lngthexc=0;
   
   A=[zeros(1,2*n) 1; extraA;
   [qbar*ones(1,2*n);qbar*ones(1,2*n)].\[M(:,2:n+1) -P;-M(:,2:n+1) R],[Winv;Winv]];

   B=[zeros(1,2*n) 1];

   C=[h; extraC; 
   [qbar;qbar].\[M(:,1).*ophw;-M(:,1).*ophwinv]];

% Now remove one of the constraints where g 
% is too large (positive or negative)

   A(1+lngthexa+indpos,:) = [];
   A(1+lngthexa+pts+indneg,:) = [];

   C(1+lngthexa+indpos,:) = [];
   C(1+lngthexa+pts+indneg,:) = [];

% Scale A and B to normalize the problem.

   D2 = max(abs([A;B])); 
   A = A./(ones(2*pts+1+lngthexa-lengthrm,1)*D2);
   B = B./D2;

   warnstate = warning;
   warning off;
  % [basic,sol,epp,lambda,tnpiv,flopcount] = mflinp(A',B',C',startbasic);
  % Get rid of FLOPCOUNT
   [basic,sol,epp,lambda,tnpiv] = mflinp(A',B',C',startbasic);
   warning(warnstate);  % put warning back to what it should be
   
   x = D2'.\(A(basic,:)\C(basic));
   oldh = h;

   if epp<=0,  
      if foundn
         hlower=h;  h=(0.5*hlower+0.5*hupper);
      else
         if n==nmax, 
            hlower=h;  h=2*h;  hupper=h;
         else 
            n=n+1;
            M=[om.*M(:,1),M];
            bold=ones(n+1,1); 
         end, % if n==nmax
         inc = floor((pts-1)/(2*n+1));
         startbasic = [5:2*inc:2*(n-1)*inc+5,...
         pts+inc+5:2*inc:pts+5+(2*n+1)*inc];
      end,% if foundn
   end, % if epp<=0,

   if epp>0,   
      a=x(1:n);
      b=[1; x(n+1:2*n)];
 
      foundn=1;

      g = polyval(a,om(1:pts))./polyval(b,om(1:pts));

% Note that we enforce one sided fits where g is too large (positive
% or negative) 

      temp = (g(1:pts)-magtmp(1:pts))./Winv(1:pts);   temp = temp(:);
      temp(indpos)=max([(-1)*temp(indpos) zeros(length(indpos),1)]')';   
      temp(indneg)=max([temp(indneg) zeros(length(indneg),1)]')';
      fit = max(abs(temp));
      hupper=fit;  h=(0.5*hupper+0.5*hlower);
      agood=a;  bgood=b;  startbasic=basic;  bold=b;
   end, % if epp>0

end % while((hupper-hlower)

% Now form the numerator and denominator polynomials in s.
% Probably need to zero protect this stuff, and check order ??

num = agood;	den = bgood;

mask = [ones(1,n);(-1)*ones(1,n)];
mask = mask(:);		mask = (-1)*mask(n:2*n)';
mask = (mask.*(omav.^((-1)*[n:-1:0])))';
den = bgood(:).*mask;
dentemp = [den';zeros(1,n+1)];
dentemp = dentemp(:);
den = dentemp(1:2*n+1)';

if n>1
   mask = [ones(1,n);(-1)*ones(1,n)];
   mask = mask(:);		mask = (-1)*mask(n+1:2*n)';
   mask = (mask.*(omav.^((-1)*[n-1:-1:0])))';
   num = agood(:).*mask;
   numtemp = [num';zeros(1,n)];
   numtemp = numtemp(:);
   num = numtemp(1:2*n-1)';
end % if n>1

% Now compute the final accuracy of the fit to g (for the 
% LP error we computed fit accuracy to g/w to check convergence).
% Note that we enforce one sided fits where g is too large (positive
% or negative) 

g            = real(polyval(num,sqrt(-1)*omegas)./polyval(den,sqrt(-1)*omegas));
temp         = ((g-magtmp).*Wdata);   temp = temp(:);
temp(indpos) = max([(-1)*temp(indpos) zeros(length(indpos),1)]')';   
temp(indneg) = max([temp(indneg) zeros(length(indneg),1)]')';
fit          = magav*max(abs(temp));

sys = magav*tf(num,den);

