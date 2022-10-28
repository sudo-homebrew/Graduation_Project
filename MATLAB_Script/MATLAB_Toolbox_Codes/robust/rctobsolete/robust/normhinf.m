function nmhinf = normhinf(varargin)
%NORMHINF Continuous H-Infinity norm.
%
% [NMHINF] = NORMHINF(A,B,C,D,tol) or
% [NMHINF] = NORMHINF(SS_,tol) computes the HINF norm of the given
%    state-space realization. Implemented here is a binary search
%    algorithm of imaginary axis eigenvalue(s) of the Hamiltonian
%
%                        -1           -1
%       H(gam) = | A + BR  D'C     -BR  B'        |
%                |       -1                -1     |
%                |C'(I+DR  D')C    -(A + BR  D'C)'|
%
%    where R = gam^2 I - D'D > 0. HINF norm equals to "gam" when H
%    has imaginary axis eigenvalue(s).
%
%    Initial guesses of the HINF norm upper/lower bounds are
%
%       Upper Bound: max_sigma(D) + 2*sum(Hankel SV(G))
%       Lower Bound: max{max_sigma(D), max_Hankel SV(G)}.
%
%    The search algorithm stops when two adjacent "gam's" have relative
%    error less than "tol". If no "tol" provided, tol = 0.001.

% R. Y. Chiang & M. G. Safonov 8/91
% Copyright 1988-2011 The MathWorks, Inc.
% All Rights Reserved.

ni = nargin;

if ni == 5 || ni == 2
   [emsg,~,~,~,a,b,c,d,tol] = mkargs5x('ss',varargin); error(emsg);
elseif ni == 4 || ni == 1
   [emsg,~,~,~,a,b,c,d] = mkargs5x('ss',varargin); error(emsg);
   tol = 1e-3;
end

sys = ss(a,b,c,d);
nmhinf = getPeakGain(sys,tol);

% LocalWords:  gam
