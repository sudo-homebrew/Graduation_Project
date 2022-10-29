function [lhs,rhs]=showlmi(LMIsys,n)
% SHOWLMI   Return the left- and right-hand-side of evaluated LMIs
%
%    [LHS,RHS]=SHOWLMI(LMISYS,N) returns the left-hand-side value LHS and 
%    right-hand-side value RHS of the N-th LMI in the LMI system LMISYS. 
%    All variables must have been preliminarily instantiated with either 
%    SETMVAR or EVALLMI.
%
%    Input:
%      LMISYS   array describing the set of evaluated LMIs (output of 
%               EVALLMI)
%      N 	    label of the selected LMI as returned by NEWLMI
%
% See also  EVALLMI, LMIINFO, LMIEDIT.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
if nargin ~= 2
   error('usage: [lhs,rhs] = showlmi(lmisys,n)');
elseif size(LMIsys,1)<10 || size(LMIsys,2)>1
  error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0)
  error('LMISYS is not an LMI description');
end

[LMI_set,~,LMI_term,data]=lmiunpck(LMIsys);


% get record on spec. LMI
lmi=LMI_set(:,LMI_set(1,:)==n);

if isempty(lmi)
   error('the argument N is out of range');
elseif isempty(LMI_term)
   lhs=zeros(lmi(2)); rhs=lhs; return
elseif any(LMI_term(4,:)~=0)
   error('Not all variables have been instantiated');
end

insize=lmi(3);
if lmi(2)>0 
   outsize=lmi(2); 
else
   outsize=insize; 
end
blckdims=lmi(7:6+lmi(6))';
blckdims=max([blckdims;ones(1,length(blckdims))]);

trange = lmi(4):lmi(5);
if isempty(trange) || ~any(trange)
   lhs=zeros(lmi(2));  rhs=lhs;  return
else
   lmit=LMI_term(:,lmi(4):lmi(5));
end

lhs=lmicte(lmit,data,blckdims,insize,n);
rhs=lmicte(lmit,data,blckdims,insize,-n);

% retrieve the outer factors and multiply
f=lmit(:,lmit(1,:)==n & lmit(2,:)==0);
if norm(lhs,1)==0
   lhs=zeros(outsize);
elseif ~isempty(f)
   % get left outer factor
   N=lmicoef(f,data);
   lhs=N'*lhs*N;
end
lhs=(lhs+lhs')/2;

f=lmit(:,lmit(1,:)==-n & lmit(2,:)==0);
if norm(rhs,1)==0
   rhs=zeros(outsize);
elseif ~isempty(f)
   % get right outer factor
   N=lmicoef(f,data);
   rhs=N'*rhs*N;
end
rhs=(rhs+rhs')/2;
