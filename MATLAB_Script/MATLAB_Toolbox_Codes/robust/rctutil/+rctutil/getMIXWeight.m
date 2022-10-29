function [W1,W3] = getMIXWeight(Gd,wc,delta)
% Converts SISO loop shape Gd into W1,W3 weights for MIXSYN.
% WC is the crossover frequency and DELTA defines the crossover band 
% as [wc/delta,delta*wc]

% Copyright 2021 The MathWorks, Inc.

% Note: Limit gain variation of W1,W3 to about 60dB for numerical
% stability. Wild gain swings make P12=[-W1*G;W3*G] difficult to 
% regularize and can cause failures for steep Gd (see g223941 example
% in tNUM_LOOPSYN)

% Parameters
GMIN = 0.5; % horizontal asymptotes at -6dB
Ts = Gd.Ts; % >0
nf = pi/Ts;

% Default
Gd = zpk(Gd); W1 = Gd;  W3 = 1/Gd;

% Compute gain of Gd vs frequency
w = [logspace(-3,-1.2,10) logspace(-1,1,41) logspace(1.2,3,10)]* wc;
if Ts>0
   w = [w(:,w<nf) nf];
end
h = freqresp(Gd,w);
g = abs(h(:));

% Horizontal asymptotes
NeedAsymptote = [g(end)<GMIN g(1)>1/GMIN];
if any(NeedAsymptote)
   if NeedAsymptote(1)
      ix = find(g>GMIN,1,'last');
      if real(h(ix))>=0
         W1 = W1 + GMIN;
      else
         W1 = W1 - GMIN;
      end
   end
   if NeedAsymptote(2)
      ix = find(g<1/GMIN,1,'first');
      if real(h(ix))>=0
         W3 = W3 + GMIN;
      else
         W3 = W3 - GMIN;
      end
   end
end

% Regularization of W1: enforce dc gain of +60dB to prevent low-frequency
% dynamics and limit gain swings
[zS,pS,kS] = zpkdata(W1,'v');
fMin = w(find(g<1e3,1,'first'));
wnz = damp(zS,Ts);
wnp = damp(pS,Ts);
nlow = sum(wnz<fMin)-sum(wnp<fMin);
if Ts==0
   zS(real(zS)>0,:) = -zS(real(zS)>0,:);
   pS(real(pS)>0,:) = -pS(real(pS)>0,:);
   zS = [repmat(-fMin,[nlow,1]) ; zS(wnz>=fMin)];
   pS = [repmat(-fMin,[-nlow,1]) ; pS(wnp>=fMin)];
else
   % Reflect unstable modes
   zSu = zS(abs(zS)>1,:);  pSu = pS(abs(pS)>1,:);
   kS = kS * real(prod(zSu)/prod(pSu));
   zS(abs(zS)>1,:) = 1./zSu;
   pS(abs(pS)>1,:) = 1./pSu;
   zMin = exp(-fMin*Ts);
   kS = kS * real(prod(1+zS(wnz<fMin))/prod(1+pS(wnp<fMin))) / (1+zMin)^nlow;
   zS = [repmat(zMin,[nlow,1]) ; zS(wnz>=fMin)];
   pS = [repmat(zMin,[-nlow,1]) ; pS(wnp>=fMin)];
end

% Regularization of W3: level off at +60dB to prevent high-frequency
% dynamics and limit gain swings
[zT,pT,kT] = zpkdata(W3,'v');
fMax = w(find(g>1e-3,1,'last'));
wnz = damp(zT,Ts);
wnp = damp(pT,Ts);
nhigh = numel(zT)-numel(pT) + sum(wnp>fMax)-sum(wnz>fMax);
if Ts==0
   zT(real(zT)>0,:) = -zT(real(zT)>0,:);
   pT(real(pT)>0,:) = -pT(real(pT)>0,:);
   kT = kT * real(prod(-zT(wnz>fMax))/prod(-pT(wnp>fMax))) * fMax^nhigh;
   zT = [repmat(-fMax,[-nhigh,1]) ; zT(wnz<=fMax)];
   pT = [repmat(-fMax,[nhigh,1]) ; pT(wnp<=fMax)];
else
   zTu = zT(abs(zT)>1,:);  pTu = pT(abs(pT)>1,:);
   kT = kT * real(prod(zTu)/prod(pTu));
   zT(abs(zT)>1,:) = 1./zTu;
   pT(abs(pT)>1,:) = 1./pTu;
   zMax = exp(-fMax*Ts);
   kT = kT * real(prod(1-zT(wnz>fMax))/prod(1-pT(wnp>fMax))) * (1-zMax)^nhigh;
   zT = [repmat(zMax,[-nhigh,1]) ; zT(wnz<=fMax)];
   pT = [repmat(zMax,[nhigh,1]) ; pT(wnp<=fMax)];
end

% Shift dynamics to create crossover band
if Ts==0
   tau = delta^(numel(zS)-numel(pS));
   zS = zS/delta;  pS = pS/delta;  kS = kS*tau;
   tau = delta^(numel(zT)-numel(pT));
   zT = zT*delta;  pT = pT*delta;  kT = kT/tau;
else
   % Set aside zeros and poles past Nyquist frequency
   wnz = damp(zS,Ts);  zkeep = (wnz<nf);  zf = zS(~zkeep);
   wnp = damp(pS,Ts);  pkeep = (wnp<nf);  pf = pS(~pkeep); 
   [zS,pS,kS] = localApplyShifts(zS(zkeep),pS(pkeep),kS,1/delta,1/delta);
   zS = [zS;zf];  pS = [pS;pf];
   % Set aside zeros and poles past Nyquist frequency
   wnz = damp(zT,Ts);  zkeep = (wnz<nf);  zf = zT(~zkeep);
   wnp = damp(pT,Ts);  pkeep = (wnp<nf);  pf = pT(~pkeep); 
   [zT,pT,kT] = localApplyShifts(zT(zkeep),pT(pkeep),kT,delta,delta);
   zT = [zT;zf];  pT = [pT;pf];
end

% Finalize weights
W1 = ss(minreal(zpk(zS,pS,kS,Ts),0.01));
W3 = ss(minreal(zpk(zT,pT,kT,Ts),0.01));

%-----------------------------------------------------------
      
function [z,p,k] = localApplyShifts(z0,p0,k,alphaZ,alphaP)
% Applies frequency shifts to discrete-time data
if isscalar(alphaZ)
   alphaZ = repmat(alphaZ,size(z0));
end
NearOne = abs(z0-1)<1e-6;  % *(1-z^alpha)/(1-z)->alpha as z->1
z = z0.^alphaZ;
tauZ = prod(alphaZ(NearOne)) * prod((1-z(~NearOne))./(1-z0(~NearOne)));
if isscalar(alphaP)
   alphaP = repmat(alphaP,size(p0));
end
NearOne = abs(p0-1)<1e-6;
p = p0.^alphaP;
tauP = prod(alphaP(NearOne)) * prod((1-p(~NearOne))./(1-p0(~NearOne)));
k = k * real(tauP / tauZ);
