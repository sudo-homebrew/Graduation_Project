function [A,B,C,D] = statescaleEngine(A,B,C,D,blk)
%

%   Copyright 1986-2020 The MathWorks, Inc.

% Possible alternative
%   1) Use c2d with tustin (bilinear transform) to map
%   problem to discrete time.
%   2) Compute D scales on [a b;c d] and absorb
%   3) Use d2c with tustin (bilinar) to map back to continuous.

nx = size(A,1);
Ms = [A B;C D];
%[~,mm] = mussv(Ms,[nx 0;abs(blk)]);  % No need for this (Agressive)
[~,mm] = mussv(Ms,[ones(nx,2);abs(blk)]);
% direct call to MUSSVUNWRAP, rather than MUSSVEXTRACT
[DLeft,DRight] = mussvunwrap(mm);
% if isreal(A) && isreal(B) && isreal(C) && isreal(D)
%    DLeft = real(DLeft);
%    DRight = real(DRight);
% end

Ms = DLeft*Ms/DRight;
A = Ms(1:nx,1:nx);
B = Ms(1:nx,nx+1:end);
C = Ms(nx+1:end,1:nx);
D = Ms(nx+1:end,nx+1:end);