function [K,CL,GAM,INFO] = hinffc(P,nY,varargin)
%HINFFC  Full-control H-infinity synthesis.
%
%   Full-control synthesis assumes the controller can directly affect both
%   the state vector x and the error signal z. This is the dual of the
%   full-information problem covered by HINFFI.
%
%   [K,CL,GAM] = HINFFC(P,NMEAS) calculates the Hoo-optimal control law
%   u = [u1;u2] = K y for the plant
%
%       dx =  A x +  B1 w + u1
%        z = C1 x + D11 w + u2
%        y = C2 x + D21 w
%
%   Here P has state-space matrices A,B1,[C1;C2],[D11;D21] and NMEAS
%   specifies the numbers of measurements y (must be the last outputs of P).
%   The gain matrix K minimizes the H-infinity norm of the closed-loop
%   transfer function from disturbance signals w to error signals z.
%
%   [K,CL,GAM] = HINFFC(P,NMEAS,GAMTRY) calculates a gain matrix K that
%   achieves the closed-loop performance level GAMTRY. If GAMTRY is not 
%   achievable, HINFFC returns GAM=Inf and K=CL=[].
%
%   [K,CL,GAM] = HINFFC(P,NMEAS,[GMIN,GMAX]) searches the range [GMIN,GMAX] 
%   for the best achievable performance GAM. HINFFC returns a gain matrix
%   K with performance 
%      * GAM <= GMIN when GMIN is achievable
%      * GMIN < GAM <= GMAX when GMAX but not GMIN is achievable
%   If GMAX itself is not achievable, HINFFC returns GAM=Inf and K=CL=[].
%
%   [K,...] = HINFFC(P,NMEAS,...,OPT) specifies additional options.
%   Use hinfsynOptions to create the option set OPT.
%
%   [K,CL,GAM,INFO] = HINFFC(P,NMEAS,...) also returns a structure INFO
%   with the following synthesis data:
%        gamma   Performance level used to compute K
%            Y   Riccati solution Yoo for this performance level
%         Preg   Regularized plant used to compute K
%
%   Reference: "State-Space Solutions to Standard H2 and Hoo Control
%   Problems", by John Doyle, Keith Glover, Pramod Khargonekar, and Bruce
%   Francis, IEEE. Trans. Automatic Control, 34 (1989), pp. 831-847.
%
%   See also HINFSYNOPTIONS, HINFFI, HINFSYN.

%   Copyright 1986-2020 The MathWorks, Inc.

% Process Inputs
narginchk(2,4)
try
   [K,CL,GAM,INFO] = hinffi(P.',nY,varargin{:});
catch ME
   if any(strcmp(ME.identifier,...
      {'Robust:design:h2syn6','Control:general:NotSupportedTimeDelayC'}))
      error(message(ME.identifier,'hinffc'))
   else
      throw(ME)
   end
end
K = K.';
CL = CL.';
INFO = struct('gamma',INFO.gamma,'Y',INFO.X,'Preg',INFO.Preg.');

