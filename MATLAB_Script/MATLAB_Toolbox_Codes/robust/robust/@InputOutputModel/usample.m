function [M,Samples] = usample(M,varargin)
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
ni = length(varargin);
WMAX = [];  Names = {};
if ni==0
   N = {1};
elseif ni==1 || isnumeric(varargin{1})
   % USAMPLE(M,N,...)
   N = varargin(1);
   if ni>2
      error(message('Control:general:InvalidSyntaxForCommand','usample','usample'))
   elseif ni==2
      WMAX = varargin{2};
   end
else
   % USAMPLE(M,Names,N,...)
   % USAMPLE(M,Names1,N1,Names2,N2,...)
   if rem(ni,2)==1
      if isnumeric(varargin{ni})
         WMAX = varargin{ni};  ni = ni-1;
      else
         error(message('Control:lftmodel:sampleblock1'))
      end
   end
   Names = varargin(1:2:ni);
   N = varargin(2:2:ni);
   % Validate names
   for ct=1:numel(Names)
      [VALID,Names{ct}] = ltipack.isNameList(Names{ct});
      if ~VALID
         error(message('Control:lftmodel:sampleblock1'))
      end
   end
end

% Apply WMAX
if ~isempty(WMAX)
   if ~(isnumeric(WMAX) && isscalar(WMAX) && isreal(WMAX) && WMAX>0)
      error(message('Robust:umodel:usample3'))
   end
   if isa(M,'ultidyn') || isa(M,'umargin')
      M.SampleMaxFrequency = WMAX;
   elseif isa(M,'ltipack.LFTModelArray')
      M = localSetMaxFreq(M,WMAX);
   end
end
      
% Ignore names that do not map to uncertain blocks
if isa(M,'ControlDesignBlock')
   Blocks = struct(getName(M),M);
elseif isa(M,'ltipack.LFTModelArray')
   Blocks = M.Blocks;
else
   Blocks = struct;
end
BlockNames = fieldnames(Blocks);
UBlockNames = BlockNames(structfun(@isUncertain,Blocks));
if isempty(Names)
   Names = {UBlockNames};
else
   for ct=1:numel(Names)
      name = Names{ct};
      if ischar(name)
         name = {name}; %#ok<*AGROW>
      else
         name = name(:);
      end
      Names{ct} = name(ismember(name,UBlockNames));  % preserves order
   end
end

% Sample uncertain blocks
args = [Names ; N];
try
   if nargout>1
      [M,Samples] = rsampleBlock(M,args{:});
   else
      M = rsampleBlock(M,args{:});
   end
catch ME
   throw(ME)
end

%------------------------

function M = localSetMaxFreq(M,WMAX)
% Enforces WMAX for ULTIDYN and UMARGIN blocks
Blocks = M.Blocks;
ix = find(structfun(@(x) isa(x,'ultidyn') || isa(M,'umargin'),Blocks));
if ~isempty(ix)
   f = fieldnames(Blocks);
   for ct=1:numel(ix)
      Blocks.(f{ix(ct)}).SampleMaxFrequency = WMAX;
   end
   M.Blocks = Blocks;
end