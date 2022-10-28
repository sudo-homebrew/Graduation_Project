function lmID=newlmi()
% NEWLMI  Add a new LMI to an LMI system
%
%   LMITAG = NEWLMI adds a new LMI to the LMI system currently described 
%   (see SETLMIS) and attaches the identifier LMITAG to it. This identifier 
%   can be used to refer to the new LMI in subsequent LMITERM, DELLMI, or 
%   SHOWLMI commands. Declaring LMIs with NEWLMI is optional and only meant
%   to make the code more readable.
%
% Output:
%  LMITAG     identifier of the new LMI.  Its value is L if
%             there are already  L-1  LMIs in the system.
%             This value remains unchanged in all subsequent
%             modifications of the LMI system
%
% See also  SETLMIS, LMIVAR, LMITERM, GETLMIS, LMIEDIT.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
% Get builder data
hLMI = LMI_BUILDER.getInstance();
GLZ_HEAD = hLMI.Header;
GLZ_LMIS = hLMI.LMIs;
if numel(GLZ_HEAD)~=10
   error('Use SETLMIS to initialize the LMI system');
end

% update GLZ_LMIS
if ~GLZ_HEAD(1) 
   lmID=1; 
else
   lmID=max(GLZ_LMIS(:,1))+1; 
end

rdim=size(GLZ_LMIS,1); nlmi=GLZ_HEAD(1);
if nlmi==rdim  % reallocate
  news=rdim+10+min(1e5,5*rdim);
  GLZ_LMIS(news,max(size(GLZ_LMIS,2),6))=0;
end

GLZ_LMIS(nlmi+1,1:7)=[lmID 0 0 0 -1 1 0];
GLZ_HEAD(1)=GLZ_HEAD(1)+1;
hLMI.Header = GLZ_HEAD;
hLMI.LMIs = GLZ_LMIS;