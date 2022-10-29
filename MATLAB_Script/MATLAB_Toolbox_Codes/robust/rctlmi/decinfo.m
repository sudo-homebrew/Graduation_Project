function decXk=decinfo(LMIsys,k)
% DECINFO   Show how matrix variables depend on decision variables
%
%   DECX = DECINFO(LMISYS,XID) displays how the matrix variable with
%   identifier XID (see LMIVAR) depends on the decision variables X1,...,XN
%   (the scalar variables optimized by the LMI solvers).
%
%   The distribution of X1,...,XN in the matrix variable X is
%   described by the integer matrix  DECX  with the following
%   convention:
%      DECX(i,j)=0   means that   X(i,j) is a hard zero
%      DECX(i,j)=n   means that   X(i,j) = xn  (n-th dec. var.)
%      DECX(i,j)=-n  means that   X(i,j) = -xn
%
%   When called with only one argument LMISYS, DECINFO works as an
%   interactive query/answer facility.
%
%   Input:
%     LMISYS     internal description of the LMI system
%     X          identifier of the variable matrix of interest
%              (see LMIVAR)
%   Output:
%     DECX       entry-wise dependence of X on x1,...,xN
%
%   See also  DECNBR, DEC2MAT, MAT2DEC, LMIVAR.

%   Authors: P. Gahinet and A. Nemirovski 3/95
%   Copyright 1995-2011 The MathWorks, Inc.
if ~any(nargin == [1,2]),
  error('usage: decinfo(lmis)  or  decX=decinfo(lmis,X)');
elseif size(LMIsys,1)<10 || size(LMIsys,2)>1,
  error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0),
  error('LMISYS is not an LMI description');
end

[~,LMI_var]=lmiunpck(LMIsys);
if isempty(LMI_var),
  error('No matrix variable described in LMISYS');
end

if nargin==1,  % interactive mode
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   k=-1;
   ndec=max(LMI_var(4,:));
   
   fprintf('\nThere are %d decision variables labeled x1 to x%d in this problem.\n\n',ndec,ndec)
   
   
   while k==-1,
      
      fprintf('\nMatrix variable Xk of interest (enter k between 1 and %d, or 0 to quit):\n',...
         size(LMI_var,2))
      
      k=input(' ?> ');
      
      if k<0 || k > size(LMI_var,2),
         k=-1;
      elseif k,
         var=LMI_var(:,k);
         klb=var(1);
         base=var(3); last=var(4);
         
         if last-base==1,
            fprintf('\n The only decision variable in X%d is x%d.\n',k,base+1)
            fprintf([' Its entry-wise distribution in X%d is as follows\n',...
               '         (0,j,-j  stand for 0,xj,-xj, respectively):\n'],k)
         else
            fprintf('\n The decision variables in X%d are among {x%d,...,x%d}.\n',...
               k,base+1,last);
            fprintf([' Their entry-wise distribution in X%d is as follows\n',...
               '         (0,j,-j  stand for 0,xj,-xj, respectively):\n'],k)
         end
         
         fprintf('\n X%d: \n\n',k)
         disp(decinfo(LMIsys,klb))
         disp('              *********');
         
         k=-1;
      end
      
   end
   
else
   %%%%
   if ~any(k==LMI_var(1,:))
      error('Unknown variable identifier X')
   end
      
   var_record=LMI_var(:,LMI_var(1,:)==k);
   if isempty(var_record), decXk=[]; return, end
   type=var_record(2);
   base=var_record(3); %last=var_record(4);
   row=var_record(5);   col=var_record(6);
   
   if type==1,      
      nblocks=var_record(7);
      vstruct=var_record(8:7+2*nblocks);
      decXk=[];
      
      for k=1:nblocks,
         siz=vstruct(2*k-1);
         if vstruct(2*k)==0,		  % scalar block
            decXk=blkdiag(decXk,(base+1)*eye(siz));
            base=base+1;
         elseif vstruct(2*k)==1,	  % full symmetric bloc
            nvar=siz*(siz+1)/2;
            decXk=blkdiag(decXk,ve2ma(base+1:base+nvar,1));
            base=base+nvar;
         else                         % zero block
            decXk=blkdiag(decXk,zeros(siz));
         end
      end
      
   elseif type==2,		% rectangular      
      decXk=ve2ma(base+1:base+row*col,2,[row,col]);
      
   else			% special      
      decXk=ve2ma(var_record(7:6+row*col),2,[row,col]);
      
   end
   
end
