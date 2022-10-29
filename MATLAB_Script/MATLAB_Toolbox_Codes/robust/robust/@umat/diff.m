function out = diff(X,N,Dim)
%DIFF  Difference operation for UMATs.
%
%   DIFF(X), for a vector X, is [X(2)-X(1)  X(3)-X(2) ... X(n)-X(n-1)].
%   DIFF(X), for a matrix X, is the matrix of row differences,
%            [X(2:n,:) - X(1:n-1,:)].
%   DIFF(X,N,DIM) is the Nth difference function along dimension DIM.
%       Here, DIM=1 corresponds to rows, DIM=2 corresponds to columns, and
%       DIM>2 corresponds to array dimensions of X.
%
%  See also UMAT.

%   Copyright 1986-2011 The MathWorks, Inc.
nin = nargin;
szX = size(X);

if nin==1
   N = 1;
   if szX(1)==1
      Dim = 2;
   elseif szX(2)==1
      Dim = 1;
   else
      Dim = 1;
   end
elseif nin~=3
   error('There must be 1 or 3 input arguments.');
end
if Dim<=2
   if Dim==1
      out = diff(eye(szX(1)),N,Dim)*X;
   elseif Dim==2
      out = X*diff(eye(szX(2)),N,Dim);
   end
elseif N==1
   ADim = Dim-2;
   idx = [ADim 1:ADim-1 ADim+1:length(szX)-2];
   szOut = szX([1 2 idx+2]);
   szOut(3) = szOut(3) - 1;
   out = umat(zeros(szOut));
   XX = permute(X,idx);
   
   for i=1:szOut(3)
      for j=1:prod(szOut(4:end))
         L1.type = '()';
         L1.subs = {':' ':' i+1 j};
         L2.type = '()';
         L2.subs = {':' ':' i j};
         out = subsasgn(out,L2,subsref(XX,L1)-subsref(XX,L2));
      end
   end
   out = ipermute(out,idx);
else
   X = diff(X,1,Dim);
   out = diff(X,N-1,Dim);
end


   