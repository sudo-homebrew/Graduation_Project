function [cm,dm,mm,info]=loopmargin(sys,blocks,ports,varargin)
%LOOPMARGIN  Comprehensive stability margin analysis.
%
%   LOOPMARGIN is obsolete, use ALLMARGIN and DISKMARGIN instead.
%
%   See also allmargin, diskmargin, linearize.

%   Copyright 2007-2018 The MathWorks, Inc.

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

narginchk(3,Inf)
[sys,blocks,ports,varargin{:}] = convertStringsToChars(sys,blocks,ports,varargin{:});

% Check for SCT
val = license('test','Simulink_Control_Design') && ~isempty(ver('slcontrol'));
if val==0
   error('Simulink LOOPMARGIN requires Simulink Control Design.');
end

% Initialize outputs
cm = []; 
dm = []; 
mm = [];
info = []; 

% Error checking on blocks/ports
if ischar(blocks)
   blocks = cellstr(blocks);
elseif ~iscellstr(blocks) %#ok<*ISCLSTR>
   error('Blocks should be a char array or a cell array of chars.');   
end

nb = size(blocks,1);
np = length(ports);
if nb~=np
   error('Number of Ports does not match the number of blocks.');
end

% Get Inputs
op =[];
opstr =[];

ni = nargin -3;
% Find the optional loopmargin argument
for i=ni:-1:1
   if isa(varargin{i},'char')
      opstr = varargin{i};
      varargin(i) = [];
   elseif isa(varargin{i},'opcond.OperatingPoint')
      op = varargin{i};
   end
end

% Blocks is a cell array of strings
for i=nb:-1:1
   lmio(i,1) = linio(blocks{i},ports(i),'outin','on');
end   

try
   load_system(sys)
catch ME
   throw(ME)
end
try
   [L,opx] = linearize(sys,lmio,varargin{:});
catch Ex
   throwAsCaller(MException('Robust:loopmargin:LinearizeError',['In the "loopmargin" command, the Simulink model %s couldn''t be'...
      ' linearized due to the following error: %s'],sys,Ex.message))
end
L = -L;

% Return operating point, linearization points 
if isa(opx,'opcond.OperatingPoint')
   info.OperatingPoint = opx;
elseif ~isempty(op)
   info.OperatingPoint = op;
else
   info.OperatingPoint = operpoint(sys);
end
info.LinearizationIO = lmio;
info.SignalNames = L.InputName;
info.L = L;

% Compute margins (classical, disk, multi-loop disk)
if size(L,1)==0 || size(L,2)==0
    % Should we error here?
    return;
else
    userin = lower(opstr( opstr~=',' & opstr~=' ' ));
    if length(userin)==1
        cm = loopmargin(L,opstr);
        dm = info;
        mm = [];
        info = [];
    elseif length(userin)==2
        [cm,dm] = loopmargin(L,opstr);
        mm = info;
        info = [];
    else
        [cm,dm,mm] = loopmargin(L);
    end
end

