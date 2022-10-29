function [VT,YT,UT,tcell] = sdlsim(sys,K,W,T,tfinal,x0,z0,int)
%SDLSIM   Time response of sampled-data feedback system.
%
% SDLSIM(P,K,W,T,TF) plots the time response of the hybrid 
% feedback system LFT(P,K) forced by the continuous input 
% signal described by W and T (values and times, as in LSIM).  
% P must be a continuous-time system, and K must be discrete-time,
% with a specified sampling time (the unspecified sampling time, 
% -1, is not allowed).  The final time is specified with TF.
%
% SDLSIM(P,K,W,T,TF,X0,Z0) specifies the initial state vector X0
% of P, and Z0 of K, at time T(1). 
%
% SDLSIM(P,K,W,T,TF,X0,Z0,INT) specifies the continuous-time
% integration step size INT.  SDLSIM forces INT = (K.Ts)/N,
% where N>4 is an integer.   If any of these optional arguments
% are omitted, or passed as empty matrices, then default values
% are used.  The default value for X0 and Z0 is zero.  Nonzero
% initial conditions are allowed for P (and/or K) only if P
% (and/or K) is an SS object.
%
% If P and/or K are LTI arrays with consistent array dimensions,
% then the time simulation is performed "pointwise" across the
% array dimensions.  
%
% [VT,YT,UT,T] = SDLSIM(P,K,W,T,TF) computes the continuous-time
% response V of the hybrid feedback system LFT(P,K) forced by the
% continuous input signal defined by W and T (values and times,
% as in LSIM).  P must be a continuous-time system, and K must be
% discrete-time, with a specified sampling time (the unspecified
% sampling time, -1, is not allowed).  The final time is
% specified with TF.  The outputs VT, YT and UT are 2-by-1 cell
% arrays: in each the first entry is a time vector, and the
% 2nd entry is the signal values.  Stored in this manner, the
% signal V can be plotted simply in two manners:
%        plot(VT{1},VT{2})     or     plot(VT{:})
% Signals Y and U are respectively the input to K and output
% of K.
% 
% If P and/or K are LTI arrays with consistent array dimensions,
% then the time simulation is performed "pointwise" across the
% array dimensions.  The outputs are 2-by-1-by-ArrayDimension cell
% arrays.  All responses can be plotted simultaneously, for example,
% plot(VT{:}).  
% 
% Optional arguments are INT (integration step size), X0 (initial
% condition for P), and Z0 (initial condition for K),
% [VT,YT,UT,T] = SDLSIM(P,K,W,T,TF,X0,Z0,INT).    SDLSIM
% forces INT = (K.Ts)/N, where N>4 is an integer.   If any of
% these arguments are omitted, or passed as empty matrices, then
% default values are used.  The default value for X0 and Z0 is 
% zero.  Nonzero initial conditions are allowed for P (and/or K)
% only if P (and/or K) is an SS object.
% 
% 
% See Also: GENSIG, LSIM, SDHINFNORM, SDHINFSYN

%   Copyright 1991-2010 The MathWorks, Inc.

% added fast ZOH code, not using LTITR just yet though
% GJW  09 Mar 1997


if nargin < 4
    error('Not enough input arguments.');
end
if nargin==4
   tfinal = [];
   x0 = [];
   z0 = [];
   int = [];
elseif nargin==5
   x0 = [];
   z0 = [];
   int = [];
elseif nargin==6
   z0 = [];
   int = [];
elseif nargin==7
   int = [];
end

if isa(sys,'DynamicSystem') && getTs(sys)==0
   sflg = ~isa(sys,'StateSpaceModel');
   sys = ss(sys);
   [nysys,nusys] = size(sys);
   nx = length(get(sys,'StateName'));
   if isempty(x0)
      x0 = zeros(nx,1);
   else
      x0 = x0(:);
      if isequal(size(x0),[1 1]) && isequal(x0,0)
         x0 = zeros(nx,1);
      elseif ~isequal(size(x0),[nx 1])
         error('x0 has incorrect state dimension.')
      end
   end
else
   error('SYS must be a continuous time system');
end
if hasdelay(sys)
   error('Plant can not have delays. See PADE for approximating delays.')
end

if any(x0) && sflg==1
    error('If SYS is a TF or ZPK, initial condition must be zero');
end

if isa(K,'DynamicSystem') && getTs(K)>0
   h = getTs(K);
   kflg = ~isa(K,'StateSpaceModel');
   K = ss(K);
   [nu,ny] = size(K); % u & y are w.r.t plant.
   nz = length(get(K,'StateName'));
   if isempty(z0)
      z0 = zeros(nz,1);
   else
      z0 = z0(:);
      if isequal(size(z0),[1 1]) && isequal(z0,0)
         z0 = zeros(nz,1);
      elseif ~isequal(size(z0),[nz 1])
         error('z0 has incorrect state dimension.')
      end
   end
   if hasdelay(K)
      K = delay2z(K);
   end
else
   error('K must be a discrete time system with specified sample time');
end 
if any(z0) && kflg==1
    error('If K is a TF or ZPK, initial condition must be zero');
end

%	set up the dimensions: w is the input vector, v is the
%	system output, y is the output to the controller and
%	u is the input from the controller.
nw = nusys - nu;
nv = nysys - ny;
if nv <= 0 || nw <= 0
   error(' Interconnection is dimensionally incorrect')
end

%   check that the input vector is compatible and extract
%   the data.
if isa(W,'double') && ismatrix(W)
   [nwpts,ncw] = size(W);
   if ncw~=nw
      error('Input W, plant P, and controller K have incompatible dimensions.');
   end
else
   error('W should be a 2-d DOUBLE.');
end
if isa(T,'double') && ismatrix(T)
   T = T(:);
   if length(T)~=nwpts
      error('W and T should have the same number of rows.');
   end
else
   error('T should be a 2-d DOUBLE.');
end

%   Now check (or select) a final time and create the output
%   time vector.  Force the simulation to start and end
%	 with an h sample.
if isempty(tfinal)
    tfinal = max(T);
elseif tfinal < min(T)
    error(' Final time less than initial time')
end
nKpts = ceil((tfinal-T(1))/h)+1;	% number of controller calcs.
tfinal = h*(nKpts-1) + T(1);
% Only keep those that matter
keep = find(T<=tfinal);
T = T(keep);
W = W(keep,:);

% REPMAT the SYS and K to get common extra dimensions
szp = size(sys);
szk = size(K);
ned = max([length(szp) length(szk)]) - 2;
szpe = [szp ones(1,ned+2-length(szp))];
szke = [szk ones(1,ned+2-length(szk))];
exd = max([szpe(3:end);szke(3:end)],[],1);
if all(szpe(3:end)==1 | szpe(3:end)==exd) && ...
  all(szke(3:end)==1 | szke(3:end)==exd)
   %XXXMINOR no SS/REPMAT
	syse = repmat(sys,[1 1 exd./szpe(3:end)]);
	Ke = repmat(K,[1 1 exd./szke(3:end)]);
   if isa(Ke,'double')
      Ke = ss([],[],[],Ke,getTs(K));
   end
else
   error('Invalid extra dimensions.');
end
ngpts = prod(exd);
vcell = cell(1,ngpts);
ycell = cell(1,ngpts);
ucell = cell(1,ngpts);
tcell = cell(1,ngpts);
for i=1:ngpts
   [vcell{i},ycell{i},ucell{i},tcell{i}] = ...
      LOCALsdlsim(syse(:,:,i),Ke(:,:,i),W,T,tfinal,int,x0,z0);
end
if nargout==0
   %XXXMAJOR need RESPPACK assistance here.
   for k=1:ny
      subplot(ny,1,k);
      for i=1:ngpts
         plot(tcell{i},vcell{i}(:,k));
         hold on;
      end
      hold off
   end
else
	VT = reshape([tcell;vcell],[2 1 exd]);
	YT = reshape([tcell;ycell],[2 1 exd]);
	UT = reshape([tcell;ucell],[2 1 exd]);
end

function [v,y,u,Tout] = LOCALsdlsim(sys,K,wdat,wt,tfinal,int,x0,z0)
h = getTs(K);
[As,Bs,Cs,Ds] = ssdata(sys);
[Ak,Bk,Ck,Dk] = ssdata(K);
ny = size(Bk,2);
nu = size(Ck,1);
szsys = size(sys);
nw = szsys(2) - nu;
nv = szsys(1) - ny;
nx = size(As,1);
nwpts = size(wdat,1);

% if an integration step size is supplied interpolate the
% input to this stepsize.  If one is not supplied then calculate
% a stepsize.
if isempty(int)
   int = 0;
end
intstep = max(0,int);
if intstep == 0
    if nwpts > 1
		wtnzdiff = diff(wt);
		wtnzdiff = wtnzdiff(abs(wtnzdiff) > eps);
		if isempty(wtnzdiff)
		    minstep = 1;
		else
		    minstep = min(abs(wtnzdiff));
		end
    else
        minstep = 1;
    end
    if isempty(As)
        intstep = minstep;
    else 
        maxeig = max(abs(pole(sys)));
        if maxeig < eps		% arbitrary in case system
			intstep = 1;		% is all integrators.
			txtandy = 'All continuous poles at ZERO, ';
			txtandy = [txtandy  'manually reset integration step size '];
			txtandy = [txtandy  'if appropriate.'];
			warning(txtandy);
        else
            intstep = 0.1/maxeig;
        end
    end
    intstep = min([intstep h/5 minstep]); % at least 5 points between controller samples.
end


%	Now check that that the integration step and controller
%	sample time are integer related.  Find that integer, N,
%	for indexing later on.  N >=2 so that we always calculate
% 	over one controller sample period.
teps = intstep*1e-8;
N = max(5,ceil(h/intstep-teps));
intstep = h/N;

if intstep ~= int
    %disp(['integration step size: ' num2str(intstep)])
end
nKpts = round((tfinal-wt(1))/h)+1;	% number of controller calcs.
vt = (wt(1):intstep:tfinal).';
npts = length(vt);


%       Now interpolate the input vector to the integration
%       step size.   The only case where this is not necessary
%       is when the stepsize is equal the spacing of a
%       regular input vector.
interflg = 0;
if npts ~= nwpts
    interflg = 1;
elseif max(abs(vt - wt)) > teps
    interflg = 1;
end
wdat = wdat.';

if interflg == 1
  wint = zeros(nw,npts);
  %disp('interpolating input vector (zero order hold)')
  if nwpts == 1
      wint = wdat*ones(1,npts);
  else
      [~,indx] = sort([wt; vt]);
      csum = ones(nwpts+npts,1);
      inew = find(indx > nwpts);
      csum(inew) = zeros(npts,1);
      csum = cumsum(csum);
      wint(:,1:npts) = wdat(:,csum(inew));
  end
  wdat = wint;
end

%	Discretize the system - the controller is already
%	discrete.
if ~isempty(As)
  ABs = [As*intstep Bs*intstep;zeros(nw+nu,nx+nw+nu)];
  eABs = expm(ABs);
  As = eABs(1:nx,1:nx);
  B1 = eABs(1:nx,nx+1:nx+nw);
  B2 = eABs(1:nx,nx+nw+1:nx+nw+nu);
  C1 = Cs(1:nv,:);
  C2 = Cs(nv+1:nv+ny,:);    
end

%	select out the appropriate parts of the D terms to
%	make the following calculations more obvious.
D11 = Ds(1:nv,1:nw);
D12 = Ds(1:nv,nw+1:nw+nu);
D21 = Ds(nv+1:nv+ny,1:nw);
D22 = Ds(nv+1:nv+ny,nw+1:nw+nu);

X = eye(nu) - Dk*D22;
if rank(X) < nu
  error(' Interconnection is ill-posed.  Check infinite freq. gains')
end
X = inv(X);
v = zeros(npts*nv+1,2);

%------------------------------------------------------------------------------%
if ~isempty(As) && ~isempty(Ak)
	yTmat = [(D22*X*Dk + eye(ny))*C2 , ... % x(k) term
             D22*X*Ck , ...                % z(k) term
             (D22*X*Dk + eye(ny))*D21 ];   % w(k) term
	uTmat = [X*Dk*C2 , ...                 % x(k) term
             X*Ck , ...                    % z(k) term
             X*Dk*D21 ];                   % w(k) term
	x = [x0 zeros(length(x0),npts-1)];
	z = [z0 zeros(length(z0),nKpts-1)];
	y = yTmat*[x(:,1);z(:,1);wdat(:,1)];	% initial y
	udat = zeros(nu,npts);		% bug fix: RSS, 31/May/95
	udat(:,1) = uTmat*[x(:,1);z(:,1);wdat(:,1)];	% and u.
	
	for i = 0:nKpts-2
        for j = 1:N-1
            udat(:,N*i+1+j) = udat(:,N*i+1);  % replicate u
            x(:,N*i+1+j) = As*x(:,N*i+j) + B1*wdat(:,N*i+j) ...
				+ B2*udat(:,N*i+j);
        end
        z(:,i+2) = Ak*z(:,i+1) + Bk*y;
        x(:,N*(i+1)+1) = As*x(:,N*(i+1)) + B1*wdat(:,N*(i+1)) ...
				+ B2*udat(:,N*(i+1));
        udat(:,N*(i+1)+1) = uTmat*[x(:,N*(i+1)+1);z(:,i+2);wdat(:,N*(i+1)+1)];
        y = yTmat*[x(:,N*(i+1)+1);z(:,i+2);wdat(:,N*(i+1)+1)];
	end
	v(1:nv*npts,1) = reshape(C1*x + [D11,D12]*[wdat;udat],nv*npts,1);
	y = zeros(npts*ny+1,2);
	y(1:ny*npts,1) = reshape(C2*x + [D21,D22]*[wdat;udat],ny*npts,1);

elseif ~isempty(As) && isempty(Ak)
	uTmat = [X*Dk*C2 , ...                 % x(k) term
             X*Dk*D21 ];                   % w(k) term
	x = [x0, zeros(length(x0),npts-1)];
	udat = zeros(nu,npts);		% bug fix: RSS, 31/May/95
	udat(:,1) = uTmat*[x(:,1);wdat(:,1)];	% initial u.
	
	for i = 0:nKpts-2
        for j = 1:N-1
            udat(:,N*i+1+j) = udat(:,N*i+1);  % replicate u
            x(:,N*i+1+j) = As*x(:,N*i+j) + B1*wdat(:,N*i+j) ...
				+ B2*udat(:,N*i+j);
        end
        x(:,N*(i+1)+1) = As*x(:,N*(i+1)) + B1*wdat(:,N*(i+1)) ...
				+ B2*udat(:,N*(i+1));
        udat(:,N*(i+1)+1) = uTmat*[x(:,N*(i+1)+1);wdat(:,N*(i+1)+1)];
	end
	v(1:nv*npts,1) = reshape(C1*x + [D11,D12]*[wdat;udat],nv*npts,1);
	y = zeros(npts*ny+1,2);
	y(1:ny*npts,1) = reshape(C2*x + [D21,D22]*[wdat;udat],ny*npts,1);

elseif isempty(As) && ~isempty(Ak)
	yTmat = [D22*X*Ck , ...                % z(k) term
             (D22*X*Dk + eye(ny))*D21 ];   % w(k) term
	uTmat = [X*Ck , ...                    % z(k) term
             X*Dk*D21 ];                   % w(k) term
	z = [z0 zeros(length(z0),nKpts-1)];
	udat = zeros(nu,npts);		% bug fix: RSS, 31/May/95
	y = yTmat*[z(:,1);wdat(:,1)];	% initial y
	udat(:,1) = uTmat*[z(:,1);wdat(:,1)];	% and u.
	
	for i = 0:nKpts-2
        for j = 1:N-1
            udat(:,N*i+1+j) = udat(:,N*i+1);  % replicate u
        end
        z(:,i+2) = Ak*z(:,i+1) + Bk*y;
        udat(:,N*(i+1)+1) = uTmat*[z(:,i+2);wdat(:,N*(i+1)+1)];
        y = yTmat*[z(:,i+2);wdat(:,N*(i+1)+1)];
	end
	v(1:nv*npts,1) = reshape([D11,D12]*[wdat;udat],nv*npts,1);
	y = zeros(npts*ny+1,2);
	y(1:ny*npts,1) = reshape(C2*x + [D21,D22]*[wdat;udat],ny*npts,1);

else		% both are constants
	vTmat = D11 + D12*X*Dk*D21;         % w(k) term
	uTmat = X*Dk*D21;                   % w(k) term
	udat = zeros(nu,npts);
	udat(:,1) = uTmat*wdat(:,1);  	   % initial u.
	
	for i = 0:nKpts-2
        for j = 1:N-1
            udat(:,N*i+1+j) = udat(:,N*i+1);  % replicate u
        end
        udat(:,N*(i+1)+1) = uTmat*wdat(:,N*(i+1)+1);
	end
	v(1:nv*npts,1) = reshape([D11,D12]*[wdat;udat],nv*npts,1);
	y = zeros(npts*ny+1,2);
	y(1:ny*npts,1) = reshape([D21,D22]*[wdat;udat],ny*npts,1);
end
%------------------------------------------------------------------------------%
v = reshape(v(1:npts*nv,1),nv,npts);
v = v';
y = reshape(y(1:npts*ny,1),ny,npts);
y = y';
u = udat';
Tout = vt;
