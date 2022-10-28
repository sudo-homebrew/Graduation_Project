function setlmis(lmi0)
% SETLMIS   Initialize the creation of a system of LMIs
%
%   SETLMIS([]) resets the internal varibales used for creating LMIs so that 
%   you can create a new system of LMIs.
%
%   SETLMIS(LMISYS0) uses an existing LMI system LMISYS0 as the starting 
%   point for creating a new system of LMIs. Subsequent commands will add 
%   to the LMI system LMISYS0 to construct a new set of LMIs.
%
%   Always use SETLMIS prior to calling LMIVAR or LMITERM.
%
%   Example:
%    %Create A'*X +X*A < 0
%    setlmis([])
%    X = lmivar(1,[2,1])
%    lmiterm([1 1 1 X],A',1,'s')
%    lmisys = getlmis;
%
%   See also  LMIVAR, LMITERM, GETLMIS.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
if nargin ~=1 || nargout ~=0
   error('usage: setlmis(lmisys0)');
end


if isempty(lmi0)
   GLZ_HEAD=zeros(10,1);
   GLZ_LMIV=[];
   GLZ_LMIS=zeros(20,7);
   GLZ_LMIT=zeros(6,50);
   GLZ_DATA=zeros(1e3,1);
else
   if size(lmi0,1)<10 || size(lmi0,2)>1
      error('LMISYS0 is not an LMI description');
   elseif length(lmi0)~=10+lmi0(1)*lmi0(4)+lmi0(2)*lmi0(5)+6*lmi0(3)+lmi0(7)
      error('LMISYS0 is not an LMI description');
   end
   [GLZ_LMIS,GLZ_LMIV,GLZ_LMIT,GLZ_DATA]=lmiunpck(lmi0);
   GLZ_LMIS = GLZ_LMIS';  % row-wise listing
   GLZ_HEAD = lmi0(1:10);
end

% Store data
hLMI = LMI_BUILDER.getInstance();
hLMI.Header = GLZ_HEAD;
hLMI.LMIs = GLZ_LMIS;
hLMI.Variables = GLZ_LMIV;
hLMI.Terms = GLZ_LMIT;
hLMI.Data = GLZ_DATA;

