function [lmit,data]=resetct(lmit,data,j,A)
%  [lmit,data]=resetct(lmit,data,j,A)
%
%  Updates the constant term lmit(:,j) by adding A to it
%
%  LOW-LEVEL FUNCTION

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.

% if nargout
% 
%   brec=lmit(5,j); lrec=lmit(6,j);
% 
%   % get value of the existing constant term
%   C=data(brec+3:brec+lrec);  % vector
%   lC=length(C); scalC=(lC==1);
% 
%   % add A
%   [rA,cA]=size(A); lA=rA*cA; scalA=(lA==1);
%   if scalC
%      if scalA
%         A = C+A;
%      else
%         E = eye(size(A));
%         A = C*E(:)+A(:);
%      end
%   else
%      if scalA
%         E = eye(round(sqrt(lC)));
%         A = A*E(:)+C;
%      else
%         A = A(:)+C;
%      end
%   end
%   
% 
%   % store result
%   if lA<=lC       % same size: simply update
%     data(brec+3:brec+lrec)=A;
%   else
%     gap=lA-lC; newl=gap+length(data);
%     data(newl)=0;
%     data(brec+3+lA:newl)=data(brec+lrec+1:length(data));
%     data(brec+1:brec+2+lA)=[rA;cA;A];
%     lmit(6,j)=2+lA;
%     % update pointers
%     ind=find(lmit(5,:)>brec);
%     lmit(5,ind)=lmit(5,ind)+gap;
%   end
% 
% else
% 
if norm(A,1)==0 
   return
end
hLMI = LMI_BUILDER.getInstance();
GLZ_HEAD = hLMI.Header;
GLZ_LMIS = hLMI.LMIs;
GLZ_LMIT = hLMI.Terms;
GLZ_DATA = hLMI.Data;
brec=GLZ_LMIT(5,j); lrec=GLZ_LMIT(6,j);

% get value of the existing constant term
C=GLZ_DATA(brec+3:brec+lrec);  % vector
lC=length(C); scalC=(lC==1);

% add A
[rA,cA]=size(A); lA=rA*cA; scalA=(lA==1);
if scalC
   if scalA
      A = C+A;
   else
      E = eye(size(A));
      A = C*E(:)+A(:);
   end
else
   if scalA
      E = eye(round(sqrt(lC)));
      A = A*E(:)+C;
   else
      A = A(:)+C;
   end
end

% store result
if lA<=lC       % same size: simply update
   GLZ_DATA(brec+3:brec+lrec)=A;
else
   gap=lA-lC; ldt=length(GLZ_DATA); newl=ldt+gap;
   GLZ_DATA(newl)=0;
   GLZ_DATA(brec+3+lA:newl)=GLZ_DATA(brec+lrec+1:ldt);
   GLZ_DATA(brec+1:brec+2+lA)=[rA;cA;A];
   GLZ_HEAD(7)=GLZ_HEAD(7)+gap;
   GLZ_LMIT(6,j)=2+lA;
   
   % update pointers
   ind=find(GLZ_LMIT(5,1:GLZ_HEAD(3))>brec);
   GLZ_LMIT(5,ind)=GLZ_LMIT(5,ind)+gap;
   
   % update block size
   row=GLZ_LMIT(2,j); col=GLZ_LMIT(3,j);
   nblcks=GLZ_LMIS(lmit,6);              % here lmit=lmirk
   blckdims=GLZ_LMIS(lmit,7:6+nblcks);
   if blckdims(row)<0
      ind=find(blckdims==blckdims(row));
      blckdims(ind)=rA*ones(1,length(ind));
   end
   if blckdims(col)<0
      ind=find(blckdims==blckdims(col));
      blckdims(col)=cA*ones(1,length(ind));
   end
   GLZ_LMIS(lmit,7:6+nblcks)=blckdims;
end

% Store data
hLMI.Header = GLZ_HEAD;
hLMI.LMIs = GLZ_LMIS;
hLMI.Terms = GLZ_LMIT;
hLMI.Data = GLZ_DATA;


