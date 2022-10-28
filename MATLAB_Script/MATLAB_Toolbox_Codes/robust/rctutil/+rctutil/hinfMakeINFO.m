function INFO = hinfMakeINFO(P,nY,nU,KBEST,dC1,dD12,dB1,dD21,Su,Sy)
% Builds INFO structure

%   Copyright 2018 The MathWorks, Inc.

% Create object
S = struct('P',P,'nY',nY,'nU',nU);
INFO = rctutil.hinfINFO(S);

% Design GAMMA level, Riccati solutions
INFO.gamma = KBEST.GAM;
INFO.X = KBEST.X;
INFO.Y = KBEST.Y;

% Controller gains
INFO.Ku = Su .* KBEST.Ku;
INFO.Kw = KBEST.Kw;
INFO.Lx = KBEST.Lx .* Sy';
INFO.Lu = Su .* KBEST.Lu .* Sy';

% Regularized plant
[A,B1,B2,C1,C2,D11,D12,D21,D22,Ts] = titodata(P,nY,nU);
[D11r,C1r,D12r,B1r,D21r] = ...
   rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12./Su',dB1,Sy.\dD21);
Preg = ss(A,[B1r B2],[C1r ; C2],[D11r D12r;D21r D22],Ts);
INFO.Preg = Preg;

% All GAMMA-suboptimal controllers
try %#ok<TRYNC>
   INFO.AS = rctutil.hinfKParam(Preg,nY,nU,KBEST.GAM,KBEST.X,KBEST.Y);
end
