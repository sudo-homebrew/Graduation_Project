function P = augw(G,W1,W2,W3)
%AUGW  Augmented plant for mixed-sensitivity design.
%
%   P = AUGW(G,W1,W2,W3) builds the augmented plant P for mixed-sensitivity 
%   design (see MIXSYN for details). Here G is the plant model and W1,W2,W3
%   are the weights on S, K*S, and T=I-S. The augmented plant P satisfies
%
%                [  W1*S  ]
%                [ W2*K*S ]  = LFT(P,K)
%                [  W3*T  ]           
%
%   and a call to MIXSYN is equivalent to
%
%      [NY,NU] = size(G)
%      K = hinfsyn(P,NY,NU)
%
%   If G has NU inputs and NY outputs, W1,W2,W3 must be square of size NY, 
%   NU, NY. Scalar or SISO values are accepted. To omit one of the terms
%   in [W1*S ; W2*K*S ; W3*T], set the corresponding weight to []. For
%   example, AUGW(G,W1,[],W3) builds P such that LFT(P,K) = [W1*S ; W3*T].
% 
%   See also MAKEWEIGHT, MIXSYN, HINFSYN, H2SYN.

% Copyright 2003-2019 The MathWorks, Inc.
narginchk(2,4)
nin = nargin;

% Check weights
[ny,nu,~] = size(G);
if isempty(W1)
   W1 = zeros(0,ny);
elseif isequal(size(W1),[1 1])
   W1 = W1 * eye(ny);
elseif ~isequal(size(W1),[ny ny])
   error(message('Robust:design:augw2','W1',int2str(ny)))
end
if nin<3 || isempty(W2)
   W2 = zeros(0,nu);
elseif isequal(size(W2),[1 1])
   W2 = W2 * eye(nu);
elseif ~isequal(size(W2),[nu nu])
   error(message('Robust:design:augw2','W2',int2str(nu)))
end
if nin<4 || isempty(W3)
   W3 = zeros(0,ny);
elseif isequal(size(W3),[1 1])
   W3 = W3 * eye(ny);
elseif ~isequal(size(W3),[ny ny])
   error(message('Robust:design:augw2','W3',int2str(ny)))
end

% Convert G to appropriate state-space type, and weights to SS
try
   G = makeStateSpace(G);
   W1 = ss.convert(W1); % watch for W1=realp or ureal
   W2 = ss.convert(W2);
   W3 = ss.convert(W3);
catch
   error(message('Robust:design:augw1'))
end
      
% Build 
%       [W1    -W1*G ]
%  P =  [0      W2   ]
%       [0      W3*G ]
%       [I     -G    ]
ey = eye(ny);
P1 = [ey;zeros(ny+nu,ny);ey];  %  [I; 0; 0; I];
P2 = [zeros(ny,nu) ; eye(nu) ; zeros(2*ny,nu)] + ...
   [-ey ; zeros(nu,ny) ; ey ; -ey] * G;
try
   % May error due to incompatible sample times, FRDs, etc
   W = blkdiag(W1,W2,W3,ey);
   P = W * [P1 P2];
catch ME
   throw(ME)
end

% Check properness and stability of weights
[isP,W] = isproper(W);
if ~(isP && isstable(W))
   error(message('Robust:design:augw3'))
end
   
% TITO partitioning
P = mktito(P,ny,nu);
