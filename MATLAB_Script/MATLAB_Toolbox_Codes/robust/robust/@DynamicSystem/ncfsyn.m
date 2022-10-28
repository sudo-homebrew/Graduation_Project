function [K,CL,gam,INFO]=ncfsyn(G,W1,W2,varargin)
%NCFSYN  Glover-McFarlane loop shaping.
%
%   [K,CL,GAM,INFO] = NCFSYN(G,W1,W2) computes the Glover-McFarlane
%   controller K for the plant G and loop-shaping weights W1,W2. Pick
%   W1,W2 so that the shaped plant Gs=W2*G*W1 has the desired loop shape.
%   NCFSYN computes the controller Ks that maximizes the robust stability
%   margin b(Gs,Ks) (see NCFMARGIN) and returns K=W1*Ks*W2 as loop-shaping
%   controller for the original plant G.
%
%             z1                          z2
%     d1 -->O--->[ W2 ]-->[ Ks ]-->[ W1 ]---->O--->[ G ]---+
%         + |    <---------------------->   + |            |
%           |                K                d2           |
%           +----------------------------------------------+
%
%   NCFSYN also returns the closed-loop CL = [I;K]*inv(I-G*K)*[I,G] from
%   [d1;d2] to [z1;z2] in the diagram above, the H-infinity performance
%   GAM, and a struct INFO with the following data:
%      INFO.gopt   optimal H-infinity performance GOPT
%      INFO.emax   nu-gap stability margin bopt=1/GOPT
%      INFO.Gs     shaped plant Gs=W2*G*W1
%      INFO.Ks     optimal controller for Gs, same as NCFSYN(Gs,1,1).
%
%   [...] = NCFSYN(G,W1,W2,TOL) specifies how tightly GAM should approximate
%   the optimal performance GOPT. The default is TOL=1e-3 (0.1% gap). Try
%   increasing TOL when the controller K has undesirable fast dynamics.
%   Very small values of TOL are likely to cause numerical difficulties.
%
%   Note:
%      * NCFSYN assumes positive feedback.
%      * Ks minimizes the H-infinity performance
%           GAM = || [I;Ks]*inv(I-Gs*Ks)*[I,Gs] ||oo
%               = || [I;Gs]*inv(I-Ks*Gs)*[I,Ks] ||oo
%               = 1 / b(Gs,Ks)
%        In general this differs from || [I;K]*inv(I-G*K)*[I,G] ||oo.
%
%   Example:
%      G = tf([1 5],[1 2 10]);
%      W1 = tf(1,[1 0]);
%      [K,CL,gam,INFO] = NCFSYN(G,W1);
%      % Compare actual and target loop shapes
%      sigma(G*K,G*W1)
%      % Verify b(Gs,Ks)=1/gam
%      b = ncfmargin(INFO.Gs,INFO.Ks,+1);
%      [b 1/gam]
%
%   See also ncfmargin, ncfmr, gapmetric, makeweight, hinfsyn.

% Copyright 2003-2021 The MathWorks, Inc.

% Reference: K. Glover and D. McFarlane, Robust stabilization of Normalized
% Coprime Factor Pnat Descriptions with H-infinity Bounded Uncertainty,
% IEEE Trans. Aut. Control, 34 (1989), pp. 821-830.
%
% The minimized Hoo norm is the norm of the closed-loop transfer from
% [w1;w2] to [z1;z2] in the following diagram:
%
%              z1        z2        <--------- Gs -------->
%     w1 -->O---->[ Ks ]---->O---->[ W1 ]-->[ G ]-->[ W2 ]---+
%         + |              + |                               |
%           |               w2                               |
%           +------------------------------------------------+
narginchk(1,4)
ni = nargin;
if ni<2 || isequal(W1,[])
   W1 = 1;
end
if ni<3 || isequal(W2,[])
   W2 = 1;
end
NOREF = (ni<4 || ~strcmp(varargin{1},'ref'));

% Factor to back off from optimal value for numerical stability
% and to avoid fast controller poles.
if NOREF && ni==4
   TOL = varargin{1};
   if ~(isnumeric(TOL) && isscalar(TOL) && isreal(TOL) && TOL>0 && TOL<Inf)
      error(message('Robust:design:ncfsyn7'))
   end
else
   TOL = 1e-3;   % Default value
end

% Build shaped plant Gs
try
   G = ss(G);  W1 = ss(W1);  W2 = ss(W2);
catch ME
   error(message('Robust:design:ncfsyn1'))
end
try
   Gs = W2 * G * W1;
catch ME
   error(message('Robust:design:ncfsyn2'))
end

% Validate Gs
[p,m,nsys] = size(Gs);
if ~isequal(iosize(G),[p m])
   error(message('Robust:design:ncfsyn3'))
elseif nsys>1
   error(message('Robust:design:ncfsyn4'))
elseif hasdelay(Gs)
   error(message('Robust:design:ncfsyn5'))
end
[isp,Gs] = isproper(Gs);
if ~isp
   error(message('Robust:design:ncfsyn6'))
end

%TOL = 1e-4; % back off from optimal value for numerical stability

if NOREF
   % NCFSYN(G,W1,W2,TOL)
   [a,b,c,d,Ts] = ssdata(Gs);
   n = size(a,1);
   
   % Solve X and Z equations in Glover-McFarlane
   % Note: -inv(Z) is anti-stabilizing solution of X equation
   rX = [eye(m) d';d -eye(p)];
   if Ts==0
      [X,~,~,INFO] = icare(a,[b zeros(n,p)],0,rX,[zeros(n,m) c']);
   else
      [X,~,~,INFO] = idare(a,[b zeros(n,p)],0,rX,[zeros(n,m) c']);
   end
   if INFO.Report>1
      K = []; CL = []; gam = Inf; INFO = []; return
   end
   rZ = [eye(p) d;d' -eye(m)];
   if Ts==0
      [Z,KZ,~,INFO] = icare(a',[c' zeros(n,m)],0,rZ,[zeros(n,p) b]);
   else
      [Z,KZ,~,INFO] = idare(a',[c' zeros(n,m)],0,rZ,[zeros(n,p) b]);
   end
   if INFO.Report>1
      K = []; CL = []; gam = Inf; INFO = []; return
   end
   L = -KZ(1:p,:)';
   
   % Optimal performance
   if n>0
      gopt = sqrt(max(real(eig(eye(n)+Z*X))));
   else
      gopt = 1;
   end
   gam = (1+TOL) * gopt;  % backoff factor
   
   % Solve equation for Xoo (modified to directly get R^(1/2)*Ux)
   % Note: Plant equations in Zhou p. 316
   r21 = [eye(m) zeros(m,p);d eye(p)/gam];
   r22 = -eye(m+p);
   if Ts==0
      R = eye(p)+d*d';
      r11 = blkdiag(zeros(m),-inv((R+R')/2));
      [Xinf,Kinf] = icare(a,[b -L/gam zeros(n,m+p)],0,...
         [r11 r21';r21 r22],[zeros(n,2*m+p) c']);
   else
      R = eye(p)+d*d'+c*Z*c';
      r11 = blkdiag(zeros(m),-inv((R+R')/2));
      [Xinf,Kinf] = idare(a,[b -L/gam zeros(n,m+p)],0,...
         [r11 r21';r21 r22],[zeros(n,2*m+p) c']);
   end
   Tx = -Kinf(1:m,:);
   Ux = -Kinf(m+1:m+p,:); % R^(1/2)*Ux for standard pencil
   
   % Controller formula for D22=0
   if Ts==0
      DK = -d'/R;  % Parrott-optimal
   else
      % DK = (I+D'*D+B'*Xoo*B)\(B'*Xoo*L-D')
      % Compute it using in least-squares formulation
      [UX,TX] = schur(Xinf); % Note: Xinf>=0
      dx = sqrt(abs(diag(TX)));
      DK = -[dx.*UX'*b;eye(m);d]\[-dx.*UX'*L;zeros(m,p);eye(p)];
   end
   BK = -L+b*DK;
   CK = Tx-DK*(c+Ux/gam);
   AK = a+L*c+b*CK;
   
   % Construct K and CL
   Ks = ss(AK,BK,CK,DK,Ts);
   if any(d(:))
      Ks = feedback(Ks,d);
   end
   K = W1*Ks*W2;
   S = feedback(eye(p+m),[zeros(p) G;K zeros(m)],+1);  % [w1;w2]->[z1;u] with u=z2+w2
   CL = S + blkdiag(zeros(p),-eye(m));

   % Verification
%    Rs = sqrtm(R);
%    P = ss(a,[-L*Rs b],[zeros(m,n);c;c],[zeros(m,p) eye(m);Rs d;Rs d],Ts); % Zhou p. 316
%    [gam hinfnorm(lft(P,Ks),1e-6)]
   
else
   % Obsolete 'ref' option
   %
   %   [K,CL,GAM,INFO] = NCFSYN(G,W1,W2,'ref') computes a 2-DOF controller K
   %   that uses the reference signal r in addition to the measurement z1.
   %   The closed-loop system CL then maps [w1; w2; r] to [z1; z2].
   %
   %             z1                          z2
   %     w1 -->O--->[ W2 ]-->[    ]-->[ W1 ]---->O--->[ G ]---+
   %         + |             [ Ks ]            + |            |
   %           |       r --->[    ]              w2           |
   %           |                                              |
   %           +----------------------------------------------+
   [Ks,gam,gopt] = localRefOption(Gs,TOL);
   if isempty(Ks)
      K = []; CL = []; gam = Inf; INFO = []; return
   else
      if issiso(W2)
         W2 = W2 * eye(p);
      end
      K = W1*Ks*blkdiag(W2,eye(m));
      % Note: r is of the same m
      OLIC = [zeros(p,2*m+p) eye(p) zeros(p,m);...
         zeros(m,p) -eye(m) zeros(m,m+p) eye(m);...
         eye(p) zeros(p,2*m+p) G;...
         zeros(m,p) eye(m) subparen(K,{':',[p+1:p+m,1:p]}) zeros(m)];
      CL = lft(OLIC,eye(m+p),p+m,p+m);
   end

end

INFO = struct('gopt',gopt,'emax',1/gopt,'Gs',Gs,'Ks',Ks);

% Verification
% S = feedback(eye(p+m),[zeros(p) Gs;Ks zeros(m)],+1);  % [w1;w2]->[z1;u] with u=z2+w2
% CLs = S + blkdiag(zeros(p),-eye(m));
% [gam getPeakGain(CLs,1e-6)]


%-----------------------------------------
function [Ks,gam,gopt] = localRefOption(Gs,TOL)
% Obsolete 'ref' option
Ts = Gs.Ts;
if Ts~=0
   Gs = d2c(Gs,'t');
end
[a,b,c,d] = ssdata(Gs);
[p,m] = size(d);
n = size(a,1);
R = eye(p)+d*d';
S = eye(m)+d'*d;
% X equation
r = [eye(m) d';d -eye(p)];
[X,~,~,INFO] = icare(a,[b zeros(n,p)],0,r,[zeros(n,m) c']);
if INFO.Report>1
   Ks = []; gam = Inf; return
end
% Z equation
r = [eye(p) d;d' -eye(m)];
[Z,~,~,INFO] = icare(a',[c' zeros(n,m)],0,r,[zeros(n,p) b]);
if INFO.Report>1
   Ks = []; gam = Inf; return
end
IZX = eye(n)+Z*X;  % I+Z*X
gopt = sqrt(max(real(eig(IZX))));
gam = (1+TOL) * gopt;
W2 = eye(n)-IZX/gam^2;
Hi = -(b*d'+W2\(Z*c'))/R;
sqrtS = sqrtm(S);
bm0 = (b-W2\(Z*c'*d))/S;
Ks = ss(a+Hi*c-bm0*b'*X,[-Hi-bm0*d',-bm0*sqrtS],-b'*X,[-d', -sqrtS]);
if Ts~=0
   Ks = c2d(Ks,abs(Ts),'t');
   Ks.Ts = Ts;
end
   

