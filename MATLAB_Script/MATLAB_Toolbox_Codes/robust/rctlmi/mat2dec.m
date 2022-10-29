function decvars=mat2dec(LMIsys,varargin)
% MAT2DEC   Construct decision variables vector from matrix variable values
%
%   DECVARS = MAT2DEC(LMISYS,X1,X2,X3,...) takes an LMI system LMISYS and
%   values X1,...,XN of its matrix variables and builds the corresponding
%   vector DECVARS of decision variable values.
%
%   See also  DEC2MAT, DECINFO, DEFCX.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
if nargin < 2
   error('usage: decvars = mat2dec(lmisys,X1,X2,X3,...)');
elseif nargin > 21
   error('MAT2DEC accepts only up to 20 matrix variables');
elseif size(LMIsys,1)<10 || size(LMIsys,2)>1
   error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0)
   error('LMISYS is not an LMI description');
end

[~,LMI_var]=lmiunpck(LMIsys);
nvars=size(LMI_var,2);
nX = nargin-1;

if nvars==0
   error('No matrix variable defined in LMISYS');
elseif nX>nvars
   error('There should be at most %d matrix variables in the calling list',nvars);
end


ldec=decnbr(LMIsys);
decvars=Inf(ldec,1);
% Inf to keep track of what is already instantiated


for k=1:nX
   var = LMI_var(:,k);
   X = varargin{k};
   type=var(2);
   base=var(3);   % base first dec var
   last=var(4);   % last dec var
   row=var(5);
   col=var(6);
   
   % check dimensioning consistency
   if isscalar(X)         % Xk = scalar
      if row==col
         X = X*eye(row);
      elseif X==0    
         X = zeros(row,col);
      else
         error(['The argument X',num2str(k),' cannot be a scalar!']);
      end
   elseif ~isequal(size(X),[row col])
      error(['The argument X',num2str(k),' is not properly dimensioned!']);
   end
   
   
   if type==1
      dec=[]; rcb=0;
      for l=1:var(7)
         bsize=var(6+2*l);
         btype=var(7+2*l);
         if btype==1      % block type = 1
            dec=[dec;ma2ve(X(rcb+(1:bsize),rcb+(1:bsize)),1)]; %#ok<*AGROW>
         elseif btype==0
            dec=[dec;X(rcb+1,rcb+1)];
         end
         rcb=rcb+bsize;
      end
      decvars(base+1:last)=dec;
   elseif type==2
      decvars(base+1:last)=ma2ve(X,2);
   else
      struct=var(7:6+row*col);
      Xvec=X';
      ind=find(abs(struct)>0);
      for ct=1:numel(ind)
         sn = struct(ind(ct));  % plus or minus dec number
         n = abs(sn);
         if isinf(decvars(n))
            decvars(n)=sign(sn)*Xvec(ind(ct)); 
         end
      end
   end
   
   % check structure consistency
   lmitmp=lmipck([],var,[],[]);
   lmitmp(8)=ldec;
   Xs=dec2mat(lmitmp,decvars,var(1));
   if norm(X-Xs,1) > 1e-8*norm(X,1)
      error(['The argument X',num2str(k),' is not properly structured']);
   end
   
end


ind=find(decvars==Inf);
decvars(ind)=zeros(length(ind),1);
