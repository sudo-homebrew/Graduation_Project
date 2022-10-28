function [C,indx1,indx2]=lmicte(terms,data,blckdims,insize,n)
% Retrieves the lhs or rhs constant term of the
% LMI of index  abs(N). Its size is given by dim.
% The output is in matrix form
%
% INDX1,INDX2 are the block coordinates

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2016 The MathWorks, Inc.
ind=find(terms(1,:)==n & terms(2,:)~=0 & terms(4,:)==0);
if isempty(ind) && nargout==1
   C=0; return,
end

nblocks=length(blckdims);
base=0;
indx1=zeros(1,nblocks);
indx2=zeros(1,nblocks);
for i=1:nblocks
   indx1(i)=base+1;
   base=base+blckdims(i);
   indx2(i)=base;
end

C=zeros(insize);
for ct=1:numel(ind)
   v = terms(:,ind(ct));
   i=v(2); j=v(3);
   cterm=lmicoef(v,data);
   if isscalar(cterm)
      cterm=cterm*eye(blckdims(i));
   end
   C(indx1(i):indx2(i),indx1(j):indx2(j))=cterm;
   if i~=j
      C(indx1(j):indx2(j),indx1(i):indx2(i))=cterm';
   end
end
