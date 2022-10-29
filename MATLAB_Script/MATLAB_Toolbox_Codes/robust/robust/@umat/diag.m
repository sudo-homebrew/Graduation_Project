function out = diag(m,k)
% DIAG Diagonal uncertain matrices and diagonals of an uncertain matrix.
%
%    If V is an uncertain vector, DIAG(V) puts V on the main diagonal.
%    If X is an uncertain matrix, DIAG(X) is the main diagonal of X.
%    Hence DIAG(DIAG(X)) is a diagonal uncertain matrix.
%
%    DIAG(V,K) when V is a vector with N components is a square matrix
%    of order N+ABS(K) with the elements of V on the K-th diagonal. K = 0
%    is the main diagonal, K > 0 is above the main diagonal and K < 0
%    is below the main diagonal. 
%  
%    DIAG(V) is the same as DIAG(V,0) and puts V on the main diagonal.
%  
%    DIAG(X,K) when X is a matrix is a column vector formed from
%    the elements of the K-th diagonal of X.
%  
%    DIAG(X) is the main diagonal of X. DIAG(DIAG(X)) is a diagonal matrix.
% 
%  Example
%     a = ureal('a',10)
%     V = [1+a 2 3 4]
%     MV = diag(V)
%  produces a diagonal matrix MV of size 4-by-4.  Alternatively, given an
%  uncertain matrix M, an uncertain vector of the diagonal  elements of M,
%  is found using DIAG.
%     M = [1+a 2; 3 4]
%     VM = diag(M)

%   Copyright 1986-2011 The MathWorks, Inc.
if nargin==1
   k=0;
elseif isempty(k)
   ctrlMsgUtils.error('Robust:transformation:diag1');
elseif isa(k,'double') && ndims(k)==2 && all(size(k)==[1 1])
   k = fix(real(k));   
else
   ctrlMsgUtils.error('Robust:transformation:diag1');
end

szm = size(m);
if szm(1)==1 || szm(2)==1   % single row or single column
   if szm(2)==1
      sz = szm(1);
   else
      sz = szm(2);
   end   
   
   out = [];
   for i=1:sz
      S(1).type='()';
      if szm(2)==1
         S(1).subs={i,1};
      else
         S(1).subs={1,i};
      end
      out = blkdiag(out,subsref(m,S));
   end

   if k>0
      out = [zeros(sz,k) out; zeros(k,k) zeros(k,sz)];
   elseif k<0
      k=abs(k);
      out = [zeros(k,sz) zeros(k,k); out zeros(sz,k)];      
   end
   
else % grab diagonal entries of a matrix
   
   out = [];
   if k>=0
      if k+1>szm(2)
         out = umat(zeros(0,1));
      else
         for i=(k+1):min([szm(2) k+szm(1)])
            S(1).type='()';
            S(1).subs={i-k,i};
            out = vertcat(out,subsref(m,S)); %#ok<*AGROW>
         end  
      end
   else
      k = abs(k);
      if k+1>szm(1)
         out = umat(zeros(0,1));
      else  
         for i=(k+1):min([szm(1) k+szm(2)])
            S(1).type='()';
            S(1).subs={i,i-k};
            out = vertcat(out,subsref(m,S));
         end            
      end
   end
   
end