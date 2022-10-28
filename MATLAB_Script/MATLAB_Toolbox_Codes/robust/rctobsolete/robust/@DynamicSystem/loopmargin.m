function [varargout] = loopmargin(varargin)
%LOOPMARGIN  Comprehensive stability margin analysis.
%
%   LOOPMARGIN is obsolete, use ALLMARGIN and DISKMARGIN instead.
%
%   See also allmargin, diskmargin, linearize.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.

%LOOPMARGIN  Comprehensive stability margin analysis of feedback loops.
%
%   LOOPMARGIN computes the stability margins of MIMO feedback systems
%   specified in either MATLAB or Simulink.
%
%   [CM,DM,MM] = LOOPMARGIN(L) analyzes the N-channel multivariable feedback
%   loop:
%
%         u --->O---->[ L ]----+---> y
%             - |              |
%               +<-------------+
%
%   The N-by-N loop transfer matrix L can be any LTI model. Negative feedback
%   is assumed in the feedback path (specify -L otherwise).
%
%   LOOPMARGIN computes three types of stability margins:
%
%   * N-by-1 structure CM of classical gain and phase margins for each
%     feedback channel with all other loops closed (loop-at-a-time
%     margins). See ALLMARGIN for details on the fields of CM
%
%   * N-by-1 structure DM of disk margins for each feedback channel with
%     all other loops closed. See DMPLOT for details on disk margins. DM
%     has the following fields:
%        GainMargin   Disk gain margin (absolute value)
%       PhaseMargin   Disk phase margin (degrees)
%         Frequency   Frequency (in rad/TimeUnit) at which the margin disk
%                     touches the Nyquist plot of the loopgain
%
%   * Structure MM of multi-loop disk margin. Multi-loop margins indicate
%     how much independent and concurrent gain and phase variations can be
%     tolerated in each feedback channel while still maintaining closed-loop
%     stability. MM has the following fields:
%        GainMargin   Sustainable amount of independent/concurrent gain
%                     variation in the feedback channels
%       PhaseMargin   Sustainable amount of independent/concurrent phase
%                     variation (in degrees).
%         Frequency   Frequency with weakest disk margin (in rad/TimeUnit).
%
%   You can compute only a subset of the output arguments CM,DM,MM by 
%   specifying a combination of the letters 'c','d','m' (for 'classical', 
%   'disk', and 'multiloop'). For example, [MM,CM] = LOOPMARGIN(L,'m,c').
%
%   [CMI,DMI,MMI,CMO,DMO,MMO,MMIO] = LOOPMARGIN(P,C) computes the stability
%   margins of the multivariable feedback loop:
% 
%         u --->O--->[ C ]-->[ P ]---+---> y
%             - |                    |         
%               +<-------------------+
% 
%   * The CMI, DMI, and MMI structures give the loop-at-a-time gain and 
%     phase margins, disk margins, and independent channel margins at 
%     the plant inputs, respectively (L = C*P)
%   * The CMO, DMO and MMO structures give the corresponding margins at
%     the plant outputs (L = P*C)
%   * The structure MMIO gives the disk margins for independent/concurrent
%     variations in all input and output channels of the feedback loop.
%   As before you can compute only a subset of the output arguments by
%   specifying a combination of the strings 'ci','di','mi','co','do','mo',
%   and 'mm' (where 'i' and 'o' refer to input and output).
%
%   The LOOPMARGIN computations can be vectorized by supplying arrays of 
%   models L or P,C.
% 
%   Example: Analyze the stability of a two-input, two-output feedback loop.
%      a = [-0.2 10;-10 -0.2]; b = eye(2); c = [1 8;-10 1]; d = zeros(2);
%      sat = ss(a,b,c,d);
%      k = [1 -2;0 1];
%      [cmi,dmi,mmi,cmo,dmo,mmo,mmio] = loopmargin(sat,k)
%      [dmi,dmo,mmio] = loopmargin(sat,k,'di,do,mm')
%
% Simulink usage:
% 
%   [CM,DM,MM,INFO] = LOOPMARGIN(MODEL,BLOCKS,PORTS) computes the stability
%   margins of a feedback loop modeled in Simulink using Simulink Control 
%   Design. MODEL specifies the name of the Simulink diagram for analysis. 
%   The cell array BLOCKS of full block paths and the vector PORTS of 
%   block port numbers jointly specify the open-loop analysis points. The
%   INFO output is a structure with data from the model linearization. 
%
%   [CM,DM,MM] = LOOPMARGIN(MODEL,BLOCKS,PORTS,OP,OPTIONS) specifies 
%   additional linearization options, see LINEARIZE for details.
%
%   See also ALLMARGIN, DMPLOT, LINIO, OPERPOINT, LOOPSENS, ROBSTAB,
%   WCMARGIN, WCGAIN.

ni = nargin;
nout = nargout;
[varargin{:}] = convertStringsToChars(varargin{:});
varargout = cell(1,nout);

% Resolve option specifier
validcode = {'ci' 'di' 'mi' 'co' 'do' 'mo' 'mm'};
if ischar(varargin{ni})
   calccode = lower(varargin{ni});
   ni = ni-1;
   calccode = regexp(calccode,'\s*,\s*','split');
   calccode = calccode(~cellfun(@isempty,calccode));
   if ni==1
      calccode = strcat(calccode,'i');
   end
   strdiff = setdiff(calccode,validcode);
   if ~isempty(strdiff)
      ctrlMsgUtils.error('Robust:analysis:InvalidOptionSpecifier',strdiff{1}(1:end-(ni==1)));
   elseif numel(unique(calccode))<numel(calccode)
      ctrlMsgUtils.error('Robust:analysis:NoRepeatOutputSpecifiers');
   end
else
   if ni==2 % P, C
      calccode = validcode;
   else % L
      calccode = validcode(1:3);
   end
end
if numel(calccode)<nout
   ctrlMsgUtils.error('Robust:analysis:OutputArgumentSpecifierMismatch');
end
   
P = varargin{1};
ny = size(P,1);
nu = size(P,2);
switch ni
   case 1
      if ny~=nu
         ctrlMsgUtils.error('Robust:analysis:NonSquareLoopGain');
      end
      C = eye(nu);
   case 2
      C = varargin{2};
      nuC = size(C,1);
      nyC = size(C,2);
      if nu~=nuC
         ctrlMsgUtils.error('Robust:analysis:OutputCInputPMismatch');
      elseif ny~=nyC
         ctrlMsgUtils.error('Robust:analysis:InputCOutputPMismatch');
      end
   otherwise
      ctrlMsgUtils.error('Robust:analysis:InvalidNumberOfInputArguments');
end
if isempty(P) || isempty(C)
   ctrlMsgUtils.error('Robust:analysis:NoEmptyModels');
end

% LOOPMARGIN performs nominal analysis
[P,C] = ltipack.matchType('',P,C);
if isa(P,'FRDModel')
   P = frd(P);  C = frd(C);
else
   P = ss(P);  C = ss(C);
end

% Vary gain/phase at inputs
cm = struct; dm = struct; mm = struct;
if any(contains(calccode,'i'))
   Li = C*P;
   if any(strcmp(calccode,'ci'))
      cm = allmargin(Li);
   end
   if any(strcmp(calccode,'mi'))
      [dm,mm] = diskmargin(Li);
   elseif any(strcmp(calccode,'di'))
      dm = diskmargin(Li);
   end
   varargout(strcmp(calccode,'ci')) = {cm};
   varargout(strcmp(calccode,'di')) = {rmExtraFields(dm)};
   varargout(strcmp(calccode,'mi')) = {rmExtraFields(mm)};
end
   
% Vary gain/phase at outputs
cm = struct; dm = struct; mm = struct;
if any(contains(calccode,'o'))
   Lo = P*C;
   if any(strcmp(calccode,'co'))
      cm = allmargin(Lo);
   end
   if any(strcmp(calccode,'mo'))
      [dm,mm] = diskmargin(Lo);
   elseif any(strcmp(calccode,'do'))
      dm = diskmargin(Lo);
   end
   varargout(strcmp(calccode,'co')) = {cm};
   varargout(strcmp(calccode,'do')) = {rmExtraFields(dm)};
   varargout(strcmp(calccode,'mo')) = {rmExtraFields(mm)};
end

% Vary gain/phase at both
loc = strcmp(calccode,'mm');
if any(loc)
   mmio = diskmargin(P,C);
   varargout{loc} = rmExtraFields(mmio);
end


function S = rmExtraFields(S)
if ~isequal(S,struct)
   S = rmfield(S,{'LowerBound','UpperBound','DiskMargin','WorstPerturbation'});
end

