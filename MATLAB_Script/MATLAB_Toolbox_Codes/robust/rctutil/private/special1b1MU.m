function [UBcert,LBcert] = special1b1MU(a,b,c,d,Ts,wTry,V,Focus)
% Assumes M is a scalar, stable, dynamical system
%
% Computes a vector of nonnegative (and possibly INF) frequency values such
% that at each one:
%    (*)  1 - freqresp(M,wNZ(i))*DeltaVal(i) == 0
%    (**) muVal(i) = 1/abs(DeltaVal(i))
% and finds all such real-valued solutions to (*).

%   Copyright 1986-2020 The MathWorks, Inc.
if isempty(Focus)
    Focus = [0 inf];
end
G = ltipack.ssdata(a,b,c,d,[],Ts);
G.Scaled = true;

% Compute mu via ALLMARGIN on G, to find negative-valued Delta's that cause
% singularity (at nonnegative frequencies, which is how allmargin always
% treats frequency).
AP = allmargin(G);
DeltaP = -AP.GainMargin;
muP = 1./AP.GainMargin;
FreqP = AP.GMFrequency;

% Compute mu via ALLMARGIN on -G, to find postive-valued Delta's that cause
% singularity.
AN = allmargin(uminus(G));
DeltaN = AN.GainMargin;
muN = 1./AN.GainMargin;
FreqN = AN.GMFrequency;

% Combine and sort results, by frequency
[wNZ,idx] = sort([FreqP FreqN]);  % freq where mu is NonZero (NZ)
muVal = [muP muN];
muVal = muVal(idx);
DeltaVal = [DeltaP DeltaN];
DeltaVal = DeltaVal(idx);

% for i=1:numel(DeltaVal)
%    LBtmp = rctutil.causeLowDamping(a,b,c,d,DeltaVal(i));
%    DeltaVal(i) = LBtmp.Delta;
%    muVal(i) = LBtmp.LB;
%    wNZ(i) = LBtmp.w;
% end

if ~any(wNZ==0)
    muVal = [0 muVal];
    DeltaVal = [rctutil.dummyDelta([-1 0]) DeltaVal];
    wNZ = [0 wNZ];
end
if ~any(isinf(wNZ))
    muVal = [muVal 0];
    DeltaVal = [DeltaVal rctutil.dummyDelta([-1 0])];
    wNZ = [wNZ inf];
end

c01 = cell(0,1);
LBcert = struct('w',c01,'LB',c01,'Delta',c01);
UBcert = struct('Interval',c01,'gUB',c01,'VLmi',c01,'ptUB',c01,'w',c01,'Jump',true);
VLmi = struct('Dr',1,'Dc',1,'Grc',0,'Gcr',0);

% [0 inf]
%    [0 small] [small big] [big inf]
% [0 w1 inf]
%    [0 small*w1] [small*w1 <w1] [<w1 >w1] [>w1 big*w1] [big*w1 inf]
zeroTol = V.abstol/100;
nonZeroTol = V.etol/100;
wFac = 0.01;
wAll = unique([wNZ(:); wTry(:)]);
nSmallInt = numel(wAll);
smallInt = zeros(nSmallInt,2);
for i=1:nSmallInt
    if i==1 % handle [0 ?small]
        wNext = wAll(2);
        if isinf(wNext)
            wNext = 0.1;
        end
        wPrevious = 0;
    elseif i==nSmallInt % handle [?big inf]
        wPrevious = wAll(i-1);
        if wPrevious==0
            wPrevious = 10;
        end
        wNext = inf;
    else % handle [<wi >wi]
        % must be intermediate... next might be inf
        wPrevious = wAll(i-1);
        wNext = wAll(i+1);
        if isinf(wNext)
            wNext = 2*wAll(i);
        end
        if wPrevious==0
            wPrevious = wAll(i)/2;
        end
    end
    
    idx = find(wAll(i)==wNZ);
    if ~isempty(idx)
        % Non-zero Mu
        [dScale,gScale,smallInt(i,:)] = nonZeroAtPoint(G,wAll(i), ...
            nonZeroTol, wFac, wPrevious, wNext);
        VLmi = struct('Dr',dScale,'Dc',dScale,'Grc',gScale,'Gcr',gScale);
        
        UBcert(end+1) = struct('Interval',smallInt(i,:),'gUB',muVal(idx)*(1 + nonZeroTol),...
           'VLmi',VLmi,'ptUB',muVal(idx),'w',wAll(i),'Jump',true);
        LBcert(end+1) = struct('w',wAll(i),'LB',muVal(idx),...
           'Delta',DeltaVal(idx));
    else
        % Zero Mu (mostly should be at 0 and inf and wTry)
        wL = wAll(i) - wFac*(wAll(i)-wPrevious);
        wR = wAll(i) + wFac*(wNext-wAll(i));
        smallInt(i,:) = [wL, wR];
        [dScale,gScale] = zeroOnInterval(G,zeroTol,wL,wR);
        VLmi = struct('Dr',dScale,'Dc',dScale,'Grc',gScale,'Gcr',gScale);

        ptw = rctutil.intervalmean([wL wR]);
        UBcert(end+1) = struct('Interval',smallInt(i,:),'gUB',zeroTol,...
           'VLmi',VLmi,'ptUB',0,'w',ptw,'Jump',true);
        LBcert(end+1) = struct('w',ptw,'LB',0,...
           'Delta',rctutil.dummyDelta([-1 0]));
    end
end

% [0 inf]
%    [0 small] [[small big]] [big inf]
% [0 w1 inf]
%    [0 small*w1] [[small*w1 <w1]] [<w1 >w1] [[>w1 big*w1]] [big*w1 inf]
for i=1:nSmallInt-1
    wL = smallInt(i,2);
    wR = smallInt(i+1,1);
    ptw = rctutil.intervalmean([wL wR]);
    [dScale,gScale] = zeroOnInterval(G,zeroTol,wL, wR);
    VLmi = struct('Dr',dScale,'Dc',dScale,'Grc',gScale,'Gcr',gScale);
    UBcert(end+1) = struct('Interval',[wL wR],'gUB',zeroTol,...
       'VLmi',VLmi,'ptUB',0,'w',ptw,'Jump',true);
    LBcert(end+1) = struct('w',ptw,'LB',0,...
       'Delta',rctutil.dummyDelta([-1 0]));
end
[~,ridx] = sort([LBcert.w]);
LBcert = LBcert(ridx);
UBcert = UBcert(ridx);

% All calculations in this function are fast.  Thus we can
% simply trim the intervals at the end based on Focus.
didx = [];
for i=1:numel(UBcert)
   I = UBcert(i).Interval;
   if I(2)<Focus(1) || I(1)>Focus(2)
      didx = [didx i];
   elseif I(1)<Focus(1) && I(2)>Focus(2)
      UBcert(i).Interval = Focus;
   elseif I(1)<Focus(1) 
      UBcert(i).Interval(1) = Focus(1);
   elseif I(2)>Focus(2)
      UBcert(i).Interval(2) = Focus(2);
   end
   if LBcert(i).w<UBcert(i).Interval(1) || LBcert(i).w>UBcert(i).Interval(2)
      ptw = rctutil.intervalmean(UBcert(i).Interval);
      LBcert(i) = struct('w',ptw,'LB',0,...
       'Delta',rctutil.dummyDelta([-1 0]));
   end
end
UBcert(didx) = [];
LBcert(didx) = [];


function [dScale,gScale] = zeroOnInterval(G,zeroTol,wL,wR)
% handles [>w(i) <w(i+1)]
% find certificates that prove mu <= zeroTol over [wL, wR] knowing that
% imag(G) is nonzero over the whole [wL,wR]
% imag(G) on [wL, inf) may not be bounded away from 0

N = 100;
Gg = fresp(G,logspace(log10(wL),log10(wR),N));
GiAbs = abs(imag(Gg(:)));
tau = min(GiAbs);
beta = max(GiAbs);
gamma = max(real(Gg(:)));
dScale = 1;
gScale = (gamma^2 + beta^2 - zeroTol^2)*dScale/(2*tau);
if imag(Gg(1))<0
    gScale = -gScale;
end
% normalize by geometric mean

function [dScale,gScale,wInt] = nonZeroAtPoint(G,wM,relTol,Fac,wPrevious,wNext)
dScale = 1;
gScale = 0;
absTol = 0.001;
M = rctutil.freqresp(G.a,G.b,G.c,G.d,G.Ts,wM);
muBound = abs(M)*(1+relTol) + absTol;

wInt = [];
N = 101;
% N2 = 50;
while isempty(wInt)
    if isinf(wM)  % [Previous>0, wM=inf, wNext=inf]
        wL = wPrevious/Fac;
        wR = inf;
    else % [Previous>=0, wM=>0/finite, wNext=finite]
        wL = wM - Fac*(wM-wPrevious);
        wR = wM + Fac*(wNext-wM);
    end
    if wL==0 %  wR must be finite
        wTest = [0 logspace(log10(wR)-4, log10(wR), N)];
    end
    if isinf(wR) % wL must be >0
        wTest = [logspace(log10(wL), log10(wL)+4, N) inf];
    end
    if wL>0 && isfinite(wR)
        wTest = logspace(log10(wL),log10(wR),N);
    end
    GTest = fresp(G,wTest);
    if all(abs(GTest(:))<muBound)
       wInt = [wTest(1) wTest(end)];
    else
       Fac = Fac/10;
    end
%     goodPoint = find(abs(GTest(:))<muBound);
%     if max(goodPoint)>N2 && min(goodPoint)<N2
%     % if max(wTest(goodPoint))>wM && min(wTest(goodPoint))<wM
%         wInt = [wTest(min(goodPoint)) wTest(max(goodPoint))];
%     else
%         Fac = Fac/2;
%     end
end
