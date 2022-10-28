function varargout=mat2lti(varargin)
% MAT2LTI
% [Y1,Y2,..,YN]=MAT2LTI(X1,X2,...,XN) deals the inputs 
% X1,...,XN onto the corresponding output Y1,...,YN, converting
% each MUTOOLS or LMI Toolbox MAT (PCK or VPCK) input to its 
% corresponding LTI representation (SS or FRD, respectively).  
%
%      Input Class		Output Format
%      SS,TF,ZPK  		MuTools PCK
%      FRD       		MuTools VPCK
%
% See also LTI2MAT

% Copyright 2003-2004 The MathWorks, Inc.

% M. G. Safonov 7/2001

varargout=cell(1,nargin);
for i=1:nargin,
	varargout{i}=m2l(varargin{i});
end

%
% ----- End of MAT2LTI.M ---- MGS 7/2001 %

%%%%%%% Begin SUBFUNCTION M2L %%%%%%%%
function sys=m2l(mat)
% SUBFUNCTION 
% SYS=M2L(MAT) Converts a single MAT (PCK, LTISS or VPCK) to LTI (SS or FRD)

% Quick return 
if ~isa(mat,'double'),
	sys=mat;
	return
end

% 
[mattype,rowd,cold,num] = minfo(mat);
switch mattype
case {'syst'}
	ssflag=logical(1);
	[a,b,c,d]=ltiss(mat);
	sys=ss(a,b,c,d);
case {'vary'}
	[mattype,rowd,cold,num] = minfo(mat);
	if isequal(mattype,'vary'),
		data=zeros(rowd,num,cold);
		[data(:),rowpoint,indv,err] = vunpck(mat);
		sys=frd(permute(data,[1 3 2]),indv);
	end
otherwise
	sys=mat;
end
%
% ---- End SUBFUNCTION M2L ---- MGS 7/2001 %
