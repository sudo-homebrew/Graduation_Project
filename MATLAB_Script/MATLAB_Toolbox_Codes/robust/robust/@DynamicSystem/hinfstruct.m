function [Result,fBest,Info,LOG] = hinfstruct(varargin)
%HINFSTRUCT  Structured H-infinity synthesis.
%
%   HINFSTRUCT tunes fixed-structure control systems using the H-infinity
%   methodology. While HINFSYN can only tune a single full-order, centralized 
%   MIMO controller, HINFSTRUCT can tune multiple fixed-order, fixed-structure 
%   compensators distributed over one or more feedback loops. To specify your 
%   control architecture:
%     1) Select the tunable components from a list of predefined Control 
%        Design blocks such as PID, gain, and fixed-order transfer function 
%        or state-space model. You can also build custom compensator 
%        structures from elementary tunable parameters, see TUNABLEBLOCK
%        for details
%     2) Use SERIES, PARALLEL, FEEDBACK, or CONNECT to build the closed-loop 
%        model as an interconnection of fixed LTI models and tunable LTI
%        components.
%   You can then use HINFSTRUCT to automatically tune the control system 
%   parameters and minimize the overall closed-loop H-infinity performance.
%   HINFSTRUCT requires some familiarity with H-infinity techniques. Use 
%   SYSTUNE and LOOPTUNE for an easier introduction to fixed-structure tuning.
%
%   [CL,GAM,INFO] = HINFSTRUCT(CL0,OPTIONS) tunes the free parameters of
%   the closed-loop model CL0 to minimize its H-infinity norm (peak 
%   input/output gain). HINFSTRUCT returns the tuned closed-loop model CL, 
%   the best closed-loop H-infinity norm GAM, and a structure INFO with 
%   additional results from the underlying optimization. The optional argument 
%   OPTIONS specifies options for the optimizer, see HINFSTRUCTOPTIONS for 
%   details.
%
%   Both CL0 and CL are generalized state-space models (see GENSS) that keep
%   track of how the tunable components interact with the plant dynamics.
%   The property CL.Blocks gives access to the tuned values. For example, 
%   if CL0 depends on two tunable blocks named C1 and C2, you can access 
%   their tuned values by typing CL.Blocks.C1 and CL.Blocks.C2. Note that
%   a given tunable parameter or component can appear multiple times in CL0.
%   See TUNABLEBLOCK and GENSS for details about tunable blocks and 
%   parametric models. Use showBlockValue(CL) to see all tuned parameter 
%   values at once, and replaceBlock(M,CL) to propagate these values to 
%   another model M with the same tunable elements (for example, the physical
%   closed-loop model without the weighting functions).
%
%   [C,GAM,INFO] = HINFSTRUCT(P,C0,OPTIONS) takes a fixed plant model P and
%   a tunable block C0 and tunes the H-infinity performance of the 
%   closed-loop model CL0 = LFT(P,C0). The output C is the tuned version of 
%   C0. To tune control architectures with several tunable blocks, set C0 
%   to a cell array of blocks (HINFSTRUCT then returns a cell array C where 
%   C{j} is the tuned version of C0{j}).
%
%   Type "demo toolbox robust" and look under "Control System Tuning" for 
%   examples.
%
%   Reference: P. Apkarian and D. Noll, "Nonsmooth H-infinity Synthesis,"
%   IEEE Transactions on Automatic Control, 51(1), pp. 71-86, 2006.
%
%   See also hinfstructOptions, tunableBlock, genss, getValue, getBlockValue,
%   showBlockValue, replaceBlock, hinfsyn, systune, looptune.

% References
% [1] P. Apkarian and D. Noll, Nonsmooth H-infinity Synthesis,
%     IEEE Transactions on Automatic Control, 51(1), pp. 71-86, 2006.
% [2] V. Bompart, P. Apkarian, and D. Noll, Control Design in the 
%     Time and Frequency Domains Using Nonsmooth Techniques.
%     System and Control Letters, 57(3), pp. 271-282, 2008.

%   Author(s): P. Apkarian, P. Gahinet
%   Copyright 2010-2012 The MathWorks, Inc.

% Parse input list
ni = nargin;
if ~isa(varargin{1},'DynamicSystem')
   error(message('Control:tuning:hinfstruct3'))
end
PCFlag = ni>1 && (iscell(varargin{2}) || isa(varargin{2},'InputOutputModel'));
if PCFlag
   % Syntax HINFSTRUCT(P,C0,OPTIONS,LOG)
   P = varargin{1};
   C = varargin{2};
   OptionalArgs = varargin(3:end);
   % Validate P
   if isParametric(P)
      error(message('Control:tuning:hinfstruct4'))
   else
      % Convert plant model P to @ss
      try
         P = ss(P,'explicit');
      catch ME
         error(message('Control:tuning:hinfstruct4'))
      end
   end
   % Validate C
   CList = iscell(C);
   if ~CList
      C = {C};
   end
   if ~all(cellfun(@(x) isa(x,'InputOutputModel') && isParametric(x),C))
      error(message('Control:tuning:hinfstruct6'))
   end
   % Build closed-loop model (to apply proper offsets and handle tunable components)
   if numel(C)>1
      try
         BlkDiagC = append(C{:});
      catch ME
         % Errors if different blocks with the same name
         throw(ME)
      end
   else
      BlkDiagC = C{1};
   end
   if any(iosize(BlkDiagC)>=fliplr(iosize(P)))
      error(message('Control:tuning:hinfstruct17'))
   end
   try
      CL = lft(P,BlkDiagC);
   catch ME
      if any(strcmp(ME.identifier,{'Control:combination:SampleTimeMismatch',...
            'Control:combination:TimeUnitMismatch'}))
         throw(ME)
      else
         error(message('Control:tuning:hinfstruct7'))
      end
   end
else
   % Syntax HINFSTRUCT(CL0,OPTIONS,LOG)
   CL = varargin{1};
   OptionalArgs = varargin(2:end);
   % Validate CL
   try
      CL = genss(CL);
   catch ME
      error(message('Control:tuning:hinfstruct26'))
   end
end

% Optional arguments
nopt = numel(OptionalArgs);
% Options
if nopt<1
   options = hinfstructOptions();
else
   options = OptionalArgs{1};
   if ~isa(options,'rctoptions.hinfstruct')
      error(message('Control:tuning:hinfstruct1'))
   end
end
% Optimization log (undocumented)
if nopt<2
   LOG = NSOptLog.Main();
else   
   LOG = OptionalArgs{2};
end

% Validate CL0 and extract problem data
if ~isfinite(CL)
   error(message('Control:tuning:hinfstruct24'))
elseif ~isParametric(CL)
   error(message('Control:tuning:hinfstruct25'))
elseif hasdelay(CL)
   error(message('Control:tuning:hinfstruct5'))
end

% Solve with SYSTUNE
wOrig = CL.InputName;
zOrig = CL.OutputName;
CL.InputName = 'w';
CL.OutputName = 'z';
if isinf(options.TargetGain)
   HinfReq = [];
else
   HinfReq = TuningGoal.Gain('w','z',1);
end
if isfinite(options.MaxFrequency)
   SpecReq = TuningGoal.Poles();
   SpecReq.MaxFrequency = options.MaxFrequency;
else
   SpecReq = [];
end
options = systuneOptions(options);
options.Hidden.Problem = 'Hinf';  % for progress display
try
   if nargout>2
      [CL,fBest,~,Info,LOG] = systune(CL,HinfReq,SpecReq,options);
      % Remap Info
      Info = localRemapInfo(Info);
   else
      [CL,fBest] = systune(CL,HinfReq,SpecReq,options);
   end
catch ME
   throw(ME)
end
if isempty(HinfReq)
   % Stabilize only -> fBest=[]
   BestInfo = getTuningInfo(CL);
   fBest = -BestInfo.MinDecay(1); % for backward compatibility
end
CL.InputName = wOrig;
CL.OutputName = zOrig;

% Set first output argument
if PCFlag
   % Return tuned blocks for HINFSTRUCT(P,C0,...) syntax
   TunedBlocks = CL.Blocks;
   TunedBlockNames = fieldnames(TunedBlocks);
   TunedBlockValues = struct2cell(TunedBlocks);
   for ct=1:numel(C)
      C{ct} = replaceB2B_(C{ct},TunedBlockNames,TunedBlockValues);
   end
   if ~CList
      C = C{1};
   end
   Result = C;
else
   % Return tuned closed-loop for HINFSTRUCT(CL0,...) syntax
   Result = CL;
end
   

%------------- Local Functions ------------------------------

function Info = localRemapInfo(SYSTUNEInfo)
% Remap Info data
nRuns = numel(SYSTUNEInfo);
Info = struct('Objective',cell(nRuns,1),'Iterations',[],'TunedBlocks',[]);
for ct=1:nRuns
   Info(ct).Objective = SYSTUNEInfo(ct).f;
   Info(ct).Iterations = SYSTUNEInfo(ct).Iterations;
   Info(ct).TunedBlocks = SYSTUNEInfo(ct).Blocks;
end

