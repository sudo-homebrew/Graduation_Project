function [info,H] = loopsens(P,C,yesInfo)
%LOOPSENS  Sensitivity functions of plant-controller feedback loop.
%
%   SF = LOOPSENS(P,C) analyzes the multivariable negative feedback loop
%   with plant P and controller C:
%
%                    u  
%          du --->O----->[ P ]-----+---> yP
%               - |                |       
%                 |             y  |
%          uC <---+<-----[ C ]-----O<--- dy
%
%   For "2-dof" architectures, C should only include the portion of the 
%   controller in the feedback path. 
%
%   SF is a structure containing the following sensitivity functions:
%       Si   Sensitivity at the plant input
%       Ti   Complementary sensitivity at the plant input (I-Si)
%       Li   Open-loop transfer at the plant inputs (CP)
%       So   Sensitivity at the plant output
%       To   Complementary sensitivity at the plant output (I-So)
%       Lo   Open-loop transfer at the plant outputs (PC)
%      PSi   Closed-loop transfer from plant input to plant output 
%      CSo   Closed-loop transfer from controller input to controller 
%            output.
%
%   See also DISKMARGIN, ROBSTAB, ROBGAIN, WCGAIN, WCDISKMARGIN.

%   Copyright 2003-2014 The MathWorks, Inc.
narginchk(2,3)

% Check size compatibility
szp = size(P);
szc = size(C);
if szc(1)~=szp(2) || szp(1)~=szc(2)
   error(message('Robust:analysis:LOOPSENS1'))
end
ny = szp(1);
nu = szp(2);

% Build closed-loop interconnection
%    H = [Si -CSo;-Ti -CSo;PSi So;PSi -To]
% Note: [-Ti -CSo] for compatibility with DISKMARGIN.
try
   % Match types
   if ~ltipack.hasMatchingType('',P,C)
      [P,C] = ltipack.matchType('',P,C);
   end
   Li = C*P;
   Lo = P*C;
   P.u = 'u';  P.y = 'yP';
   % Positive feedback loop with -C
   C = -C;  C.u = 'y';  C.y = 'uC';
   Sum1 = sumblk('u = du + uC',nu);
   Sum2 = sumblk('y = dy + yP',ny);
   % Beware of simplifying unstable modes
   opt = connectOptions('Simplify',false);
   H = connect(P,C,Sum1,Sum2,{'du','dy'},{'u','uC','y','yP'},opt);
catch ME
   throw(ME)
end

% Build info structure
if nargin<3 || yesInfo
   % Assess stability
   try
      if isstable(H)
         Stable = 1;
      else
         Stable = 0;
      end
      Poles = pole(H);
   catch  %#ok<*CTCH>
      Stable = NaN;
      Poles = NaN;
   end
   info = struct('Si',subparen(H,{1:nu,1:nu}),...
      'Ti',-subparen(H,{nu+1:2*nu,1:nu}),'Li',Li,...
      'So',subparen(H,{2*nu+1:2*nu+ny,nu+1:nu+ny}),...
      'To',-subparen(H,{2*nu+ny+1:2*(nu+ny),nu+1:nu+ny}),...
      'Lo',Lo,'PSi',subparen(H,{2*nu+ny+1:2*(nu+ny),1:nu}),...
      'CSo',-subparen(H,{nu+1:2*nu,nu+1:nu+ny}),...
      'Poles',Poles,'Stable',Stable);
else
   info = [];
end
