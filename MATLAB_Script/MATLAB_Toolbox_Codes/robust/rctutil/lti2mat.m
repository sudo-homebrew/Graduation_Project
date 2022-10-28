function varargout=lti2mat(varargin)
% LTI2MAT
% CEL=LTI2MAT(X1,X2,...,XN) deals the inputs 
% X1,...,XN onto the cell array CEL={Y1,...,YN}, converting
% each LTI (SS,TF,ZPK, and FRD) input to its corresponding 
% MUTOOLS or LMI Toolbox MAT representation (PCK,VPCK).  
%
% Input Class		Output Format
% SS,TF,ZPK			MuTools PCK
% FRD				MuTools VPCK
%
% This function, together with MAT2LTI allows easy creation 
% of LTI compatible versions of MuTools and LMI Toolbox 
% functions.
%
% CAVEATS: If a TF or ZPK is present,
% then at least one of the other inputs must be an SS; furthermore,
% each ZPK and TF input must have a proper transfer function.

% Copyright 2003-2004 The MathWorks, Inc.

% M. G. Safonov 7/2001

ssflag=logical(0);
tfflag=logical(0);
varargout=cell(1,nargin);
for i=1:nargin,
	[varargout{i},ssflag,tfflag]=l2m(varargin{i},ssflag,tfflag);
end
if tfflag & ~ssflag,
	error('At least one LTI input must be an SS')
	% This is required, because LTI protocol does not allow SS to return when only ZPK's or TF's are input
end
%
% ----- End of LTI2MAT.M ---- MGS 7/2001%

%%%%%%% SUBFUNCTION L2M %%%%%%%%
function [mat,ssflag,tfflag]=l2m(sys,ssflag,tfflag)
% [mat,ssflag,tfflag]=l2m(sys,ssflag,tfflag)
% Converts LTI SYS to equivalent MuTools or LMI Toolbox MAT
switch class(sys)
case {'ss'}
	ssflag=logical(1);
	ssflag=logical(1);
	[a,b,c,d,e]=dssdata(sys);
	if isequal(e,eye(size(e))),
		mat=pck(a,b,c,d);
	else
	    mat=ltisys(a,b,c,d,e);
	end
case { 'tf' , 'zpk' }
	if isproper(sys);
		[a,b,c,d]=ssdata(sys);
		mat=pck(a,b,c,d);
		tfflag=logical(1);
	else
		error( ['Class ' class(sys) ' transfer function input must be proper'])
	end
case {'frd'}
	% mat=frd2vpck(sys);
	[temp,w] = frdata(sys);
	[r,c,d]=size(temp);
	data=zeros(r*d,c);
	temp=permute(temp,[1 3 2]);
	data(:)=temp(:);
	mat=vpck(data,w);
otherwise
	mat=sys;
end
%
% ----- End of L2M.M ---- MGS 7/2001 %
