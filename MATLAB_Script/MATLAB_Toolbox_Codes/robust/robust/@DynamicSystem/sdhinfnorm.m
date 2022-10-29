function [gaml,gamu] = sdhinfnorm(sdsys,k,delay,tol)
%SDHINFNORM Sample-data H-infinity norm of a feedback system.
%
%[GAML,GAMU] = SDHINFNORM(SYS,KD) computes the L2 induced norm of a
%continuous-time LTI system, SYS, in feedback with a discrete-time
%LTI system, KD, connected through an ideal sampler and a zero-order
%hold. The continuous-time plant SYS is partitioned:
%                      | a   b1   b2   |
%            SYS    =  | c1   0    0   |
%                      | c2   0    0   |
%
%Note that d must be zero. GAML and GAMU are lower and upper bounds
%on the induced norm.
%
%[GAML,GAMU] = SDHINFNORM(SYS,KD,DELAY,TOL) allows the inclusion of
%a computational delay, DELAY, which must be a non-negative integer
%defining the number of computational delays of the controller. The
%default value of DELAY is 0. TOL is the tolerance on the difference
%between upper and lower norm bounds when the search terminates
%scaled by the lower bound, i.e. (GAMU-GAML)<TOL*GAMU. TOL must be
%between 1e-5 and 0.1, its default value is 0.001.
%	                _________
%	               |         |
%	     z <-------|  sdsys  |<-------- w
%	               |         |
%	      /--------|_________|<-----\
%	      |       __  		         |
%	      |      |d |		            |
%	      |  __  |e |   ___          |
%	      |_|S |_|l |__| K |_________|
%	        |__| |a |  |___|
%	             |y |
%	             |__|
%
%See also: NORM, H2SYN, HINFSYN, SDLSIM and SDHINFSYN.

%   Copyright 1991-2020 The MathWorks, Inc.


%perform type checking
if nargin < 2
   error('Not enough input arguments.');
end

% setup default cases
if nargin == 2
   delay = 0;
   tol=0.001;
elseif nargin == 3
   tol=0.001;
end

if ~(isa(sdsys,'DynamicSystem') && sdsys.Ts==0)
   error('SDSYS must be a continuous time system.');
elseif hasdelay(sdsys)
   error('Plant can not have delays. See PADE for approximating delays.')
end
try
   sdsys = ss(sdsys);
catch
   error('SDSYS must have a state-space realization.');
end

if ~(isa(k,'DynamicSystem') && k.Ts>0)
   error('K must be a discrete time system with positive sample time.');
elseif hasdelay(k)
   k = delay2z(k);
end
try
   k = ss(k);
catch
   error('K must have a state-space realization.');
end

szt = size(tol);
if ~(isa(tol,'double') && szt(1)==szt(2) && szt(1)==1 && tol>=1e-5 ...
      && tol<=0.1)
   warning(message('Robust:analysis:TolReset'))
   tol = 0.001;
end

if delay < 0 || (ceil(delay)~=floor(delay))
   error(' DELAY must be a non-negative integer');
end

szp = size(sdsys);
szk = size(k);
ned = max([length(szp) length(szk)]) - 2;
szpe = [szp ones(1,ned+2-length(szp))];
szke = [szk ones(1,ned+2-length(szk))];
exd = max([szpe(3:end);szke(3:end)],[],1);

if all(szpe(3:end)==1 | szpe(3:end)==exd) && ...
      all(szke(3:end)==1 | szke(3:end)==exd)
   sdsyse = repmat(sdsys,[1 1 exd./szpe(3:end)]);
   ke = repmat(k,[1 1 exd./szke(3:end)]);
   if isa(ke,'double')
      ke = ss(ke,'Ts',get(k,'Ts'));
   end
else
   error('Invalid extra dimensions.');
end

gaml = zeros([1 1 exd]);
gamu = zeros([1 1 exd]);
ngpts = prod(exd);
for i=1:ngpts
   % LOCALsdhinfnorm(sdsyse(:,:,i),ke(:,:,i),delay,tol);
   [gaml(1,1,i),gamu(1,1,i)] = LOCALsdhinfnorm(...
      subsref(sdsyse,substruct('()',{':' ':' i})),...
      subsref(ke,substruct('()',{':' ':' i})),...
      delay,tol);
end


function [gaml,gamu]=LOCALsdhinfnorm(sdsys,k,delay,tol)

[p,m] = size(sdsys);
[a,b,c,d] = ssdata(sdsys);
n = size(a,1);

h = get(k,'Ts');
%k = ss(k);
[ncon,nmeas] = size(k);

% check sampled data feedback system is stable.
sdsys_sh = c2d(sdsys,h);
k_del=k;
if delay>0
   k_del.InputDelay = delay;
   k_del = delay2z(k_del);
end
cl_sh = lft(sdsys_sh,k_del);
cl_poles = pole(cl_sh);
if any(abs(cl_poles)>=1)
   %disp('sampled data system unstable');
   gaml = inf; gamu = inf;
   return,
end

% check data
if rank( diag( exp(eig(a)*h) + ones(n,1) ) ) < n
   disp(' SYSTEM is pathologically sampled');
   gaml = nan; gamu = nan;
   return
end
if max(max(abs(d)))~=0
   disp(' D matrix of sdsys is non-zero');
   gaml = nan; gamu = nan;
   return
end

% get an initial guess for gamma from the norm of the
% discrete time system, assuming piecewise constant inputs
% and sampled error signals.
tmptol = 0.1;
gamtmp=(1+tmptol)*norm(d2c(cl_sh,'tustin'),inf,tmptol);
%%dis_norm=hinfnorm(bilinz2s(cl_sh,h),0.1);
%%gamtmp=dis_norm(2); % changed from taking the lower bound:  gamtmp=dis_norm(1);

% main gamma iteration loop
foundgaml=0; foundgamu=0; gamu=1; gaml=0; compnormltgaml=0;
amp=1.1;

while ~foundgaml || ~foundgamu || ((gamu-gaml)>tol*gamu)
   b1=b(1:n,1:m-ncon)/sqrt(gamtmp);
   b2=b(1:n,m-ncon+1:m)*sqrt(gamtmp);
   c1=c(1:p-nmeas,1:n)/sqrt(gamtmp);
   c2=c(p-nmeas+1:p,1:n)*sqrt(gamtmp);
   [T,invT,S,s]=ham2schr(a,b1,c1,.02/h);
   
   if ~compnormltgaml
      compnormtmp=compnorm(T,invT,S,s,h);
   else
      compnormtmp=1;
   end %if ~compnormltgaml
   
   if ~compnormtmp
      foundgaml=1;
      gaml=gamtmp;
      if foundgamu
         gamtmp=(gaml+gamu)/2;
      else
         gamtmp=amp*gaml;
         amp=amp^2;  % to increase the rate
      end %if foundgamu
   else  % (i.e. compnormtmp=1)
      % XXXX
      soln= sdn_step(a,b1,b2,c1,c2,T,invT,S,s,h,delay,k/gamtmp);
      if soln==0
         compnormltgaml=1;
         foundgaml=1;
         gaml=gamtmp;
         if foundgamu
            gamtmp=(gaml+gamu)/2;
         else
            gamtmp=amp*gaml;
            amp=amp^2;
         end %if foundgamu
      else %(i.e. soln==1)
         foundgamu=1;
         gamu=gamtmp;
%         if foundgamu,
            gamtmp=(gaml+gamu)/2;
%          else
%             gamtmp=gamu/2;
%          end, %if foundgamu
      end % if soln==0
   end % if ~compnormtmp
end %while ~foundgaml

if nargout == 0
   disp(['norm between ' num2str(gaml) ' and ' num2str(gamu)]);
end
