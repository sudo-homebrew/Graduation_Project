function [Sx,Su,Sy,dB1,dC1,dD12,dD21,XINFO,YINFO] = regScaleXY(A,B1,B2,C1,C2,D11,D12,D21,Ts,GAMXYE,OPTS)
%

%   Copyright 1986-2020 The MathWorks, Inc.
XINFO.P12 = ss(A,B2,C1,D12,Ts);
YINFO.P12 = ss(A,B1,C2,D21,Ts);

% Compute initial regularization that is independent of state scaling
% and limits the size of the perturbation dP12 of P12
GAM = max(1e-2,2*GAMXYE);   
[dC1,dD12,XINFO.dP12] = localRegP12(A,B2,C1,D12,Ts);
[dB1,dD21,YINFO.dP12] = localRegP12(A',C2',B1',D21',Ts);
dB1 = dB1';   dD21 = dD21';
nd2 = size(dB1,2);
ne2 = size(dC1,1);
C1R = [C1 ; dC1];  D12R = [D12 ; dD12];
B1R = [B1 , dB1];  D21R = [D21 , dD21];
D11R = blkdiag(D11,zeros(ne2,nd2));
if OPTS.Instrument
   XINFO.XSENS1 = hinfricsens(A,[],B1R/GAM,B2,C1R,D11R/GAM,D12R,Ts,{1e-7,1e7},0,false);
   YINFO.XSENS1 = hinfricsens(A',[],C1R'/GAM,C2',B1R',D11R'/GAM,D21R',Ts,{1e-7,1e7},0,false);
end

% Scale regularized model
[Sx0,Su0,Sy0] = localScaleXY(A,B1R,B2,C1R,C2,D11R,D12R,D21R,Ts,GAM);
AS = lrscale(A,Sx0,1./Sx0);
B1S = lrscale(B1R,Sx0,[]);
B2S = lrscale(B2,Sx0,Su0);
C1S = lrscale(C1R,[],1./Sx0);
C2S = lrscale(C2,Sy0,1./Sx0);
D12S = lrscale(D12R,[],Su0);
D21S = lrscale(D21R,Sy0,[]);
if OPTS.Instrument
   XINFO.XSENS1S = hinfricsens(AS,[],B1S/GAM,B2S,C1S,D11R/GAM,D12S,Ts,{1e-7,1e7},0,false);
   YINFO.XSENS1S = hinfricsens(AS',[],C1S'/GAM,C2S',B1S',D11R'/GAM,D21S',Ts,{1e-7,1e7},0,false);
end

% Compute minimalist regularization in scaled coordinates
B1S = lrscale(B1,Sx0,[]);
C1S = lrscale(C1,[],1./Sx0);
D12S = lrscale(D12,[],Su0);
D21S = lrscale(D21,Sy0,[]);
[dC1,dD12] = rctutil.regularizeH2(AS,B2S,C1S,D12S,Ts,1,OPTS,'P12');
[dB1,dD21] = rctutil.regularizeH2(AS',C2S',B1S',D21S',Ts,1,OPTS,'P21');
dB1 = dB1';   dD21 = dD21';
C1SR = [C1S ; dC1];  D12SR = [D12S ; dD12];
B1SR = [B1S , dB1];  D21SR = [D21S , dD21];
nd2 = size(dB1,2);
ne2 = size(dC1,1);
D11SR = blkdiag(D11,zeros(ne2,nd2));
if OPTS.Instrument
   XINFO.XSENS2 = hinfricsens(AS,[],B1SR/GAM,B2S,C1SR,D11SR/GAM,D12SR,Ts,{1e-7,1e7},0,false);
   XINFO.dP12R = ss(AS,lrscale(B2S,[],1./Su0),dC1,lrscale(dD12,[],1./Su0));
   YINFO.XSENS2 = hinfricsens(AS',[],C1SR'/GAM,C2S',B1SR',D11SR'/GAM,D21SR',Ts,{1e-7,1e7},0,false);
   YINFO.dP12R = ss(AS,dB1,lrscale(C2S,1./Sy0,[]),lrscale(dD21,1./Sy0,[]));
end
% undo scaling
dC1 = lrscale(dC1,[],Sx0);
dD12 = lrscale(dD12,[],1./Su0);
dB1 = lrscale(dB1,1./Sx0,[]);
dD21 = lrscale(dD21,1./Sy0,[]);

% Scale again for final regularization
[Sx,Su,Sy] = localScaleXY(AS,B1SR,B2S,C1SR,C2S,D11SR,D12SR,D21SR,Ts,GAM);
AS = lrscale(AS,Sx,1./Sx);
B1S = lrscale(B1SR,Sx,[]);
B2S = lrscale(B2S,Sx,Su);
C1S = lrscale(C1SR,[],1./Sx);
C2S = lrscale(C2S,Sy,1./Sx);
D12S = lrscale(D12SR,[],Su);
D21S = lrscale(D21SR,Sy,[]);
if OPTS.Instrument
   XINFO.XSENS2S = hinfricsens(AS,[],B1S/GAM,B2S,C1S,D11SR/GAM,D12S,Ts,{1e-7,1e7},0,false);
   YINFO.XSENS2S = hinfricsens(AS',[],C1S'/GAM,C2S',B1S',D11SR'/GAM,D21S',Ts,{1e-7,1e7},0,false);
end
Sx = Sx0 .* Sx;
Su = Su0 .* Su;
Sy = Sy0 .* Sy;

%--------------------------------------------------
function [dC1,dD12,dP12] = localRegP12(A,B2,C1,D12,Ts)

P12 = ss(A,B2,C1,D12);
p = eig(A);
wmax = 1+max(abs(p));
[nx,nu] = size(B2);

% Pick regularization of D12 to
% 1) prevent dynamics far beyond wmax
% 2) ensure rho(A) * ||inv([0 D12';D12 -I])|| < 1/eps where
%    rho(A) is a proxy for the best-case value of ||MX|| after scaling
delta = 1e-3 * localColNorm(evalfr(P12,1i*wmax));
delta = max(delta,sqrt(eps*wmax));
dD12 = [zeros(nx,nu) ; diag(delta)];

% Evaluate P12 response away from poles
f = wmax * logspace(-8,0,9)';
dist = abs(1i*f-p.')./(1+f);
f = f(min(dist,[],2)>1e-2);
nf = numel(f);

delta = 1e-2 * norm(C1,1) * ones(1,nx);
h1 = freqresp(P12,f);
h2 = freqresp(ss(A,B2,eye(nx),0),f);
for ct=1:nf
   rnorm = localColNorm(h2(:,:,ct)');
   delta = min(delta,0.01*norm(h1(:,:,ct),1)./rnorm);
end
dC1 = [diag(delta) ; zeros(nu,nx)];

% Validation
dP12 = ss(A,B2,dC1,dD12);



function [Sx,Su,Sy,INFO] = localScaleXY(A,B1,B2,C1,C2,D11,D12,D21,Ts,GAM)
% Scales states and controls to minimize sensitivity of X Riccati equations.
nX = size(A,1);  nX2 = 2*nX;
[nE,nD] = size(D11);
nU = size(B2,2);
nY = size(C2,1);
E = eye(nX);

% Pick frequency grid for sensitivity minimization
[fmin,fmax] = localDynRange(A,Ts);
lfmin = log10(fmin);  lfmax = log10(fmax);
fgrid = logspace(lfmin,lfmax,ceil(lfmax-lfmin));
if Ts==0
   cfreqs = complex(0,[0 fgrid Inf]);
else
   cfreqs = exp(complex(0,[0 fgrid*Ts pi]));
end
nf = numel(cfreqs);

% X equation
m = nU+nE+nD;
if Ts==0
   MX = [zeros(nX) A B2 B1/GAM zeros(nX,nE);A' zeros(nX,nX+nU+nD) C1';...
      B2' zeros(nU,nX+nU+nD) D12';B1'/GAM zeros(nD,nX+nU) -eye(nD) D11'/GAM;...
      zeros(nE,nX) C1 D12 D11/GAM -eye(nE)];
   NX = blkdiag([zeros(nX) E;-E' zeros(nX)],zeros(m));
else
   MX = [zeros(nX) A B2 B1/GAM zeros(nX,nE);-E' zeros(nX,nX+nU+nD) C1';...
      zeros(nU,nX2+nU+nD) D12';zeros(nD,nX2+nU) -eye(nD) D11'/GAM;...
      zeros(nE,nX) C1 D12 D11/GAM -eye(nE)];
   NX = [[zeros(nX) E;-A' zeros(nX);-B2' zeros(nU,nX);...
      -B1'/GAM zeros(nD,nX);zeros(nE,nX2)] zeros(nX2+m,m)];
end
if Ts==0
   M1 = abs(MX);
else
   M1 = abs(MX)+abs(NX);
end
% Construct N1
N1 = zeros(size(M1));
for ct=1:nf
   if isinf(cfreqs(ct))
      % REVISIT: scale R independently
      Zi = blkdiag(zeros(2*nX),ltipack.util.safeMinv(MX(nX2+1:end,nX2+1:end)));
   else
      Z = MX-cfreqs(ct)*NX;
      if Ts~=0
         Z(:,1:nX) = Z(:,1:nX)/cfreqs(ct);  % make Hermitian
      end
      Zi = ltipack.util.safeMinv(Z);
   end
   N1 = N1 + abs(Zi);
end

% Y equation
m = nY+nE+nD;
if Ts==0
   MY = [zeros(nX) A' C2' C1'/GAM zeros(nX,nD);A zeros(nX,nX+nY+nE) B1;...
      C2 zeros(nY,nX+nY+nE) D21;C1/GAM zeros(nE,nX+nY) -eye(nE) D11/GAM;...
      zeros(nD,nX) B1' D21' D11'/GAM -eye(nD)];
   NY = blkdiag([zeros(nX) E';-E zeros(nX)],zeros(m));
else
   MY = [zeros(nX) A' C2' C1'/GAM zeros(nX,nD);-E zeros(nX,nX+nY+nE) B1;...
      zeros(nY,nX2+nY+nE) D21 ; zeros(nE,nX2+nY) -eye(nE) D11/GAM;...
      zeros(nD,nX) B1' D21' D11'/GAM -eye(nD)];
   NY = [[zeros(nX) E';-A zeros(nX);-C2 zeros(nY,nX);...
      -C1/GAM zeros(nE,nX);zeros(nD,nX2)] zeros(nX2+m,m)];
end
if Ts==0
   M2 = abs(MY);
else
   M2 = abs(MY)+abs(NY);
end
% Construct N2
N2 = zeros(size(M2));
for ct=1:nf
   if isinf(cfreqs(ct))
      % REVISIT: scale R independently
      Zi = blkdiag(zeros(2*nX),ltipack.util.safeMinv(MY(nX2+1:end,nX2+1:end)));
   else
      Z = MY-cfreqs(ct)*NY;
      if Ts~=0
         Z(:,1:nX) = Z(:,1:nX)/cfreqs(ct);  % make Hermitian
      end
      Zi = ltipack.util.safeMinv(Z);
   end
   N2 = N2 + abs(Zi);
end

% Scale
[Sx,Su,Sy,INFO] = rctutil.HSCALE(nX,nU,M1,N1,nY,M2,N2);



function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end

function [fmin,fmax] = localDynRange(A,Ts)
% Computes dynamic range
wn = damp(A,Ts);
wn = wn(wn>1e-6);
if isempty(wn)
   fmin = 1e-6;   fmax = 1e6;
else
   fmin = 0.1*min(wn);   fmax = 10*max(wn);
end
