function lmis=getlmis()
% GETLMIS   Get the internal description of the LMI system
%
%   LMISYS = GETLMIS returns a computer description LMISYS of the currently
%   described LMI system (see SETLMIS). The description of an LMI system
%   starts with SETLMIS and ends with GETLMIS.
%
%   The internal representation LMISYS can be passed
%   directly to the LMI solvers or any other LMI-Lab
%   function.
%
%   Example:
%    %Create A'*X +X*A < 0
%    setlmis([])
%    X = lmivar(1,[2,1])
%    lmiterm([1 1 1 X],A',1,'s')
%    lmisys = getlmis;
%
%   See also  SETLMIS, LMIVAR, LMITERM.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.

% Get builder data
hLMI = LMI_BUILDER.getInstance();
GLZ_HEAD = hLMI.Header;
GLZ_LMIS = hLMI.LMIs;
GLZ_LMIV = hLMI.Variables;
GLZ_LMIT = hLMI.Terms;
GLZ_DATA = hLMI.Data;

if numel(GLZ_HEAD)~=10
   error('The LMI system must be first described with SETLMIS, LMIVAR, and LMITERM')
end

nlmi=GLZ_HEAD(1);
nvar=GLZ_HEAD(2);
nterm=GLZ_HEAD(3);
ldt=GLZ_HEAD(7);
lmis=lmipck(GLZ_LMIS(1:nlmi,:)',GLZ_LMIV(:,1:nvar),...
   GLZ_LMIT(:,1:nterm),GLZ_DATA(1:ldt));

% Clear builder data
hLMI.Header = [];
hLMI.LMIs = [];
hLMI.Variables = [];
hLMI.Terms = [];
hLMI.Data = [];


