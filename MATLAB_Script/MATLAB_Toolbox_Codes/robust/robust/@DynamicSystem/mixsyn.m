function varargout = mixsyn(G,varargin)
%MIXSYN  H-infinity mixed-sensitivity design.
%
%   For the feedback loop
%                   
%                    e       u
%           r --->O--->[ K ]-->[ G ]---+---> y
%               - |                    |
%                 +<-------------------+
%  
%   the mixed-sensitivity design seeks a controller K that minimizes
%
%                  ||  W1*S  ||
%            GAM = || W2*K*S ||
%                  ||  W3*T  ||oo
%   where
%     * S = inv(I+G*K) is the sensitivity function
%     * K*S is the transfer from r to u (control effort)
%     * T = I-S = G*K/(I+G*K) is the complementary sensitivity
%     * W1,W2,W3 are weights chosen to emphasize certain frequency bands.
%       For example, to enforce low sensitivity and good tracking at low
%       frequency, W1 should have high gain at low frequency and low gain
%       at high frequency.
%
%   [K,CL,GAM,INFO] = MIXSYN(G,W1,W2,W3) computes the controller K
%   minimizing the mixed-sensitivity cost GAM above. CL is the closed-loop
%   transfer [W1*S ; W2*K*S ; W3*T] and INFO contains data from the
%   underlying H-infinity synthesis (see HINFSYN).
%
%   [K,CL,GAM,INFO] = MIXSYN(G,W1,W2,W3,GAMTRY) returns a controller K
%   satisfying GAM<=GAMTRY (if one exists). When W1,W2,W3 capture the
%   desired limits on the gains of S,K*S,T, use GAMTRY=1 to just enforce
%   these limits.
%
%   [K,CL,GAM,INFO] = MIXSYN(G,W1,W2,W3,[GMIN,GMAX]) limits the search for
%   the best performance GAM to the range [GMIN,GMAX].
%
%   [K,CL,GAM,INFO] = MIXSYN(G,W1,W2,W3,...,OPT) specifies additional
%   options for the H-infinity synthesis. Use hinfsynOptions to create OPT.
%
%   If G has NU inputs and NY outputs, W1,W2,W3 must be square of size NY,
%   NU, NY. Scalar or SISO values are accepted. To omit one of the terms
%   in the cost GAM, set the corresponding weight to [].
%
%   Example
%      G = ss(-1,2,3,4);  % plant to be controlled 
%      wc = 10;   % desired closed-loop bandwidth
%      A = 1000;  % desired disturbance attenuation inside bandwidth
%      M = 2;     % max sensitivity (peak gain of S)
%      W1 = makeweight(A,wc,1/M);
%      W2 = [];   % omit W2*K*S term
%      W3 = makeweight(1/M,wc,A);
%      [K,CL,GAM] = mixsyn(G,W1,W2,W3);
%
%      % Plot design results:
%      L = G*K;  % loop transfer function
%      S = feedback(1,L); % sensitivity
%      T = 1-S;  % complementary sensitivity
%      sigma(S,'b',GAM/W1,'b--',T,'r',GAM/W3,'r--')
%
%   See also MAKEWEIGHT, AUGW, HINFSYN.

%   Copyright 2003-2019 The MathWorks, Inc.
narginchk(2,Inf)
% Check G
[ny,nu,nsys] = size(G);
if nsys~=1
   error(message('Robust:design:mixsyn1'))
end
% Build augmented plant
try
   P = augw(G,varargin{1:min(3,nargin-1)});
catch ME
   throw(ME)
end
% H-infinity synthesis
try
   [varargout{1:nargout}] = hinfsyn(P,ny,nu,varargin{4:end});
catch ME
   throw(ME)
end