function [A,B,flag]=nncoeff(t,data,task)
%    LOW-LEVEL FUNCTION   [A,B,flag]=nncoeff(t,data,task)
%
%    Retrieves the coefficient(s) associated with a LMI term T
%
%    TASK=0 (variable term AXB) ->
%              returns A and B' in row-first storage, or equivalently 
%              A' and B in column-first storage
%    TASK=1 (scalar var. term x*(A*B) ) ->
%              returns A*B in row-first storage, or equivalently
%              (A*B)' in column-first storage
%    TASK=2 (outer factor N) ->
%              returns N' row-first storage, or equivalently N 
%              in column-first storage
%
%    FLAG is 1 for self-conjugated terms in diagonal blocks

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2016 The MathWorks, Inc.

shft=t(5);        % base of data segment in DATA

if task==2   % outer fact
   rA=data(shft+1); cA=data(shft+2);
   shft=shft+2;
   A = data(shft+1:shft+rA*cA);  % N column first
   B=[]; flag=0;
else
   % Get A
   rA=data(shft+1); cA=data(shft+2);
   shft=shft+2; lA=rA*cA;
   A = reshape(data(shft+1:shft+lA),[rA,cA]); % A column first
   % Get B
   shft=shft+lA;
   rB=data(shft+1); cB=data(shft+2);
   shft=shft+2; lB=rB*cB;
   % B column first, same as B' row first 
   Bvec = data(shft+1:shft+lB);
   if task==1
      % scalar variable
      X = (A*reshape(Bvec,[rB,cB]))';
      A = X(:);  % A*B in row-first storage
      B = [];
   else
      A = reshape(A',[lA 1]);  % A row first
      B = Bvec;
   end
   flag = data(shft+1+lB);
end