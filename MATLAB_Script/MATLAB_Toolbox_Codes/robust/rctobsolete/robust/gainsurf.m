function K = gainsurf(name,Kinit,varargin)
%GAINSURF  Tunable gain surface for gain scheduling.
%
%   NOTE: GAINSURF is obsolete. Use tunableSurface instead.
%
%   GAINSURF models a gain surface of the form
%      K(SV) = K0 + K1 * F1(SV) + ... + KM * FM(SV)
%   where
%      * SV is a vector of scheduling variables
%      * F1(.),...,FM(.) are user-defined functions of SV
%      * K0,K1,...,KM are tunable scalar- or matrix-valued coefficients.
%   This gain surface represents one of the gains in a gain-scheduled 
%   controller. Use SYSTUNE to tune such gain-scheduled controllers for a 
%   set of operating conditions.
%
%   K = GAINSURF(NAME,KINIT,F1,...,FM) contructs a tunable model of the
%   gain surface
%      K(SV) = K0 + K1 * F1(SV) + ... + KM * FM(SV) .
%   Given a grid of design points SV and the arrays F1,...,FM of function 
%   values F1(SV),...,FM(SV) at these design points, GAINSURF returns an
%   array of gain values K(SV) parameterized by the tunable coefficients 
%   K0,...,KM (see GENMAT). The string NAME specifies the gain block name
%   and KINIT specifies the initial value of the constant coefficient K0.
%   You can combine K with other static or dynamic elements to model the 
%   closed-loop system, use TuningGoal objects to specify the requirements 
%   at each design point, and use SYSTUNE to tune the coefficients K0,...,KM 
%   against these requirements.
%
%   Note: For best results, it is recommended to 
%     1. Use a set of design points SV representative of the operating range
%     2. Normalize the scheduling variables SV so that they range in the 
%        interval [-1,1].
%
%   Example 1: You must schedule the scalar gain K as a function of incidence  
%   angle "alpha" (ranging between 0 and 15 degrees) and speed V (ranging  
%   between 300 and 600 m/s). You decide to try a gain surface of the form
%      K(alpha,V) = K0 + K1 * alpha + K2 * V + K3 * (alpha*V)
%   To tune its coefficients K0,K1,K2,K3, pick 16 design points linearly 
%   spaced in alpha and V:
%      [alpha,V] = ndgrid(0:5:15,300:100:600)
%   and use GAINSURF to model the gain surface at these design points:
%      F1 = alpha
%      F2 = V
%      F3 = alpha .* V
%      K = gainsurf('K',1,F1,F2,F3)
%      K.SamplingGrid.alpha = alpha;
%      K.SamplingGrid.V = V;
%   This creates a 4-by-4 array of scalar gains with tunables blocks 
%   'K_0','K_1','K_2','K_3' for the coefficients K0,K1,K2,K3.
%
%   Example 2: To improve solver performance, further normalize the values 
%   of alpha and V used in the gain surface of Example 1:
%       alphaN = alpha/15   % ranges in [0,1]
%       VN = (V-450)/150    % ranges in [-1,1]
%       K = gainsurf('K',1,alphaN,VN,alphaN.*VN)
%       K.SamplingGrid.alpha = alpha;
%       K.SamplingGrid.V = V;
%   The resulting gain surface is now expressed in terms of the normalized
%   variables alphaN and VN:
%       K(alphaN,VN) = K0 + K1 * alphaN + K2 * VN + K3 * (alphaN*VN) .
%   Remember to account for this normalization when implementing the 
%   gain-scheduled controller.
%
%   See also tunableSurface, systune, TuningGoal.

%   Author(s): P. Gahinet
%   Copyright 1984-2013 The MathWorks, Inc.

% Validate NAME and KINIT
narginchk(2, Inf)
if ~isvarname(name)
   error(message('Control:tuning:gainsurf1'))
end
if ~(isnumeric(Kinit) && ismatrix(Kinit) && isreal(Kinit)) || hasInfNaN(Kinit)
   error(message('Control:tuning:gainsurf2'))
end
[ny,nu] = size(Kinit);

% Construct K
M = numel(varargin);
if M==0
   K = realp(name,Kinit);
else
   F1 = varargin{1};
   sF = size(F1);
   
   % Check Fm arrays
   for m=1:M
      Fm = varargin{m};
      if ~(isnumeric(Fm) && isreal(Fm) && all(isfinite(Fm(:))))
         error(message('Control:tuning:gainsurf3',num2str(m)))
      elseif ~isequal(size(Fm),sF)
         error(message('Control:tuning:gainsurf4',num2str(m)))
      end
   end
   
   % Build K
   K = realp(sprintf('%s_0',name),Kinit);
   for m=1:M
      K = K + realp(sprintf('%s_%d',name,m),zeros(ny,nu)) * ...
         reshape(double(varargin{m}),[1 1 sF]);
   end
end
