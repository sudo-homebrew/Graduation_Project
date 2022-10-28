function Samples = usample(UVars,N,varargin)
%USAMPLE   Generates random samples of uncertain matrices or systems.
% 
%   B = USAMPLE(A,N) picks N random samples of the uncertainty in A and 
%   returns these samples in an array B of size [SIZE(A) N].  A can be 
%   an uncertain element, matrix, or system (see UMAT, USS, UFRD,...).
%   USAMPLE(A) is the same as USAMPLE(A,1).
%
%   [B,SAMPLES] = USAMPLE(A,N) also returns the uncertainty samples in 
%   the N-by-1 structure array SAMPLES whose field names and values are 
%   the names and sample values of the uncertain elements in A. Note that
%   B = usubs(A,SAMPLES).
%
%   [B,SAMPLES] = USAMPLE(A,NAMES,N) samples only the uncertain elements 
%   listed in the string or string vector NAMES. The output B is uncertain 
%   unless NAMES includes all uncertain elements.  Any entry of NAMES that 
%   does not appear in A is ignored.  
%
%   [B,SAMPLES] = USAMPLE(A,NAMES1,N1,NAMES2,N2,...) takes N1 samples of
%   the uncertain elements listed in NAMES1, N2 samples of the uncertain
%   elements listed in NAMES2, and so on. The resulting B has size 
%   [SIZE(A) N1 N2 ...].
%
%   To further constrain the frequency range of ULTIDYN dynamics, use the
%   syntax:
%      [B,SAMPLES] = USAMPLE(A,N,WMAX),
%      [B,SAMPLES] = USAMPLE(A,NAMES,N,WMAX) 
%      [B,SAMPLES] = USAMPLE(A,NAMES1,N1,NAMES2,N2,WMAX)
%   If A is a continuous-time USS or UFRD, gain-bounded ULTIDYN elements
%   are sampled so that all their poles have magnitude less than WMAX.
%   All other cases are reduced to this case by bilinear transformation.
%
%   SAMPLES = USAMPLE(UVARS,N) generates N random samples for the uncertain 
%   variables listed in the structure UVARS. The fields of UVARS specify 
%   the uncertainty as UREAL, UCOMPLEX, or ULTIDYN objects and the fields  
%   of SAMPLES contain the uncertainty samples. Together with UFIND, 
%   this syntax is useful to sample the uncertainty in a Simulink model 
%   (see USIM_DEMO) or a collection of uncertain matrices or systems. 
%   As before, USAMPLE(UVARS) is equivalent to USAMPLE(UVARS,1) and 
%   USAMPLE(UVARS,N,WMAX) further constrains ULTIDYN dynamics. 
%
%   USAMPLE is useful for Monte-Carlo analysis of uncertain systems.
%
%   Example 1: 
%      % Create an uncertain model of P(s) = g/(tau*s+1) where
%      % g varies in [3,5] and tau = 0.5 +/- 30%
%      g = ureal('g',4);
%      tau = ureal('tau',.5,'Percentage',30);
%      P = tf(g,[tau 1]);
%      % Create an integral controller based on nominal (g,tau)
%      C = tf(1,[1 0]) / (2*tau.Nominal*g.Nominal)
%      % Plot the plant and closed-loop responses for 20 samples
%      % of (g,tau)
%      subplot(2,1,1); step(usample(P,20))
%      subplot(2,1,2); step(usample(feedback(P*C,1),20))
%
%   Example 2:
%      % Create 4 uncertain parameters a,b,c,d and form M=[a b;c d]
%      a=ureal('a',1); b=ureal('b',2); c=ureal('c',3); d=ureal('d',4);
%      M = [a b;c d];
%      % Pick 5 samples for (a,b), 3 samples for (c,d), and evaluate
%      % M at the corresponding 5x3=15 combinations
%      M1 = usample(M,{'a';'b'},5,{'c';'d'},3)
%      % Evaluate M at 15 samples of (a,b,c,d)
%      M2 = usample(M,{'a';'b';'c';'d'},15)
%
%   See also USUBS, UFIND, USS, UFRD, UMAT, UREAL, UCOMPLEX, ULTIDYN.

%   Copyright 2003-2011 The MathWorks, Inc.

% Parse input list
ni = nargin;
if ni<1 || ~isstruct(UVars)
   error(message('Robust:umodel:usample1'))
end
if ni<2
   N = 1;
elseif ~(isnumeric(N) && isscalar(N) && isreal(N) && N>0 && N==round(N))
   error(message('Control:lftmodel:sampleblock2'))
end
if ni>2
   wMax = varargin{1};
   if ~(isnumeric(wMax) && isscalar(wMax) && isreal(wMax) && wMax>0)
      error(message('Robust:umodel:usample3'))
   end
end

% Instantiate each atom
UNames = fieldnames(UVars);
UVals = {};
for ct=1:length(UNames)
   uvar = UVars.(UNames{ct});
   if ~isa(uvar,'UncertainBlock')
      error(message('Robust:umodel:usample4')   )
   else
      [~,S] = usample(uvar,N,varargin{:});
   end
   UVals = cat(1,UVals,struct2cell(S));
end
Samples = cell2struct(UVals,UNames,1);
