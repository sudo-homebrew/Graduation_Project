function X=dec2mat(LMIsys,decvars,k)
% DEC2MAT   Extract matrix variable value from vector of decision variables
%
%   X = DEC2MAT(LMISYS,DECVARS,XID) returns the value of the matrix 
%   variable with identifier XID (see LMIVAR) given the LMI system 
%   description LMISYS and the vector DECVARS of decision variable values.
%   This command is useful to convert the output of LMI solvers back into
%   individual matrix variables.
%
%   See also  MAT2DEC, DECINFO, DECNBR, LMIVAR.

%   Authors: P. Gahinet and A. Nemirovski 3/95
%   Copyright 1995-2011 The MathWorks, Inc.
if nargin ~= 3,
  error('usage: X = dec2mat(LMISYS,decvars,k)');
elseif size(LMIsys,1)<10 || size(LMIsys,2)>1,
  error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0),
  error('LMISYS is not an LMI description');
end

decvars=decvars(:);
decn=decnbr(LMIsys);
if length(decvars)~=decn,
  error('DECVARS must be a vector of length %d',decn)
end

[~,LMI_var]=lmiunpck(LMIsys);

if isempty(LMI_var),
  error('No matrix variable description in LMISYS');
elseif isempty(k),
  error('K must be nonempty');
elseif k<=0 || k>max(LMI_var(1,:)),
  error('K is out of range');
end


varrec=LMI_var(:,LMI_var(1,:)==k);
type=varrec(2);
base=varrec(3); last=varrec(4);
row=varrec(5);   col=varrec(6);
lfreev=length(decvars);


if type==1,
   
   if base>=lfreev || last>lfreev,
      error('Length of DECVARS too short!');
   end
   
   nblocks=varrec(7);
   vstruct=varrec(8:7+2*nblocks);
   decvars=decvars(base+1:last);
   first=1; X=[];
   
   for k=1:nblocks,
      siz=vstruct(2*k-1);
      if vstruct(2*k)==0,		% scalar block
         X=blkdiag(X,decvars(first)*eye(siz,siz));
         first=first+1;
      elseif vstruct(2*k)==1,     % full symmetric bloc
         nvar=siz*(siz+1)/2;
         X=blkdiag(X,ve2ma(decvars(first+(0:nvar-1)),1));
         first=first+nvar;
      else                       % zero block
         X=blkdiag(X,zeros(siz));
      end
   end
   
elseif type==2,		% rectangular
   
   if base>=lfreev || last>lfreev,
      error('Length of DECVARS too short!');
   end
   
   X=ve2ma(decvars(base+(1:row*col)),2,[row,col]);
   
else			% special
   
   vstruct=varrec(7:6+row*col);
   
   if any(vstruct>lfreev)
      error('Length of DECVARS too short!');
   end
   
   decvars=[0 ; decvars];
   X=decvars(abs(vstruct)+ones(size(vstruct)));
   X=ve2ma(sign(vstruct).*X,2,[row,col]);
   
end
