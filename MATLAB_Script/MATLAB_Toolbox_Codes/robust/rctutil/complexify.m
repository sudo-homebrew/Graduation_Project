function Mc = complexify(M,alpha,flag)
%COMPLEXIFY  Replace UREAL atoms by a summation of UREAL and UCOMPLEX atoms. 
%   MC = COMPLEXIFY(M,ALPHA) replaces each UREAL atom in M with a summation
%   of a UREAL atom and a UCOMPLEX atom, effectively replacing pure real
%   uncertainty with a real+complex uncertainty.
%
%   Each UREAL atom in MC has the same Name and NominalValue as the 
%   corresponding UREAL atom in M.  If RANGE is the range of one UREAL atom 
%   from M, then the range of the corresponding UREAL atom in MC is
%            [RANGE(1)+ALPHA*DIFF(RANGE)/2  RANGE(2)-ALPHA*DIFF(RANGE)/2]
%   In other words, the range is reduced by equal amounts at each end, and
%   ALPHA represents (in a percentage sense) the reduction in the total
%   range. The UCOMPLEX atom will add this reduction in range back 
%   into MC, but as a ball with real and imaginary parts.
%
%   The UCOMPLEX atom has NominalValue of 0, and Radius equal to
%   ALPHA*DIFF(RANGE)/2.  Its name is the name of the original UREAL atom,
%   appended with the characters '_cmpxfy'.
%
%   COMPLEXIFY can be used to improve the conditioning of robust stability
%   calculations (ROBSTAB, MUSSV) for situations when there are 
%   predominantly UREAL uncertain elements.  If M is a USS or UFRD, then
%   the typical usage would be as follows:
%
%   % Complexify problem
%   MC = COMPLEXIFY(M,ALPHA);
%
%   % Peform robustness analysis on Complexified Uncertain Model
%   [RSBnd,DestabC] = ROBSTAB(MC);
%
%   % Extract approximately-destabilizing Real values of Uncertainty from
%   % the destabilizing complexified version
%   AlmostDestab = ICOMPLEXIFY(DestabC);    
%
%   MC = COMPLEXIFY(M,ALPHA,'ULTIDYN') replaces each UREAL atom in M with
%   a summation of a UREAL atom and a ULTIDYN atom, effectively replacing
%   pure real uncertainty with a real+unmodeled dynamics uncertainty.
%   The ULTIDYN atom has a norm bound equal to ALPHA*DIFF(RANGE)/2. 
%
%   See also ICOMPLEXIFY, MUSSV, ROBSTAB.

%   Copyright 2003-2011 The MathWorks, Inc.
narginchk(2,3)
if alpha>=0.5 || alpha<0
   ctrlMsgUtils.error('Robust:analysis:Complexify1')
end
if nargin==2
   flag = 'ucomplex';
end
unc = M.uncertainty;
fn = fieldnames(unc);

Mc = M;
ASOrig = cell(length(fn),1);

% Save all AutoSimplify values, and then temporarily turn all to 'off'
for i = 1:length(fn)
   val = unc.(fn{i});
   ASOrig{i} = val.AutoSimplify;
   Mc.Uncertainty.(fn{i}).AutoSimplify = 'off';
end

% Loop through, making all substitutions
cSuffix = '_cmpxfy';
for i = 1:length(fn)
   val = Mc.Uncertainty.(fn{i});
   if isa(val,'ureal')
      range = val.Range;
      cm = alpha*(range(2)-range(1))/2;
      newrang = [range(1)+cm, range(2)-cm];
      val.Range = newrang;
      if strncmpi(flag,'ul',2)
         cval=ultidyn([fn{i} cSuffix],[1 1],'Bound',cm,'AutoSimplify','off');
      else
         cval=ucomplex([fn{i} cSuffix],0,'radius',cm,'AutoSimplify','off');
      end
      Mc = usubs(Mc,fn{i},val+cval);
   end
end

% Restore AutoSimplify values (copying the UREAL value into its
% corresponding UCOMPLEX value.
for i = 1:length(fn)
   val = Mc.Uncertainty.(fn{i});
   Mc.Uncertainty.(fn{i}).AutoSimplify = ASOrig{i};
   if isa(val,'ureal')
      appname = [fn{i} '_cmpxfy'];
      Mc.Uncertainty.(appname).AutoSimplify = ASOrig{i};
   end
end
