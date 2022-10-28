% LTIARRAY2USS   Computes uncertain system bounding given LTI SS array
%
% USYS = LTIARRAY2USS(P,PARRAY,ORD) calculates an uncertain system
% USYS with nominal value P, and whose range of behavior includes
% the given array of systems, PARRAY.  USYS employs an input 
% multiplicative uncertainty model, of the form
%         USYS = P*(I + WT*ULTIDYN('Name',[size(P,2) size(P,2)]))
% where WT is a stable scalar system, whose magnitude overbounds
% the relative difference, (P-PARRAY)/P.  The state dimension of WT is ORD.
% Both P and PARRAY must be in the classes SS/TF/ZPK/FRD.  If P is an FRD,
% then USYS will be a UFRD object, otherwise USYS will be a USS object.
% The name of the ULTIDYN atom is based on the variable name of PARRAY
% in the calling workspace.
%
% [USYS,WT] = LTIARRAY2USS(P,PARRAY,ORD) returns the weight WT used
% to bound the infinity norm of (P-PARRAY)/P.  
%
% [USYS,WT] = LTIARRAY2USS(P,PARRAY,ORD,'OutputMult') uses multiplicative
% uncertainty at the plant output (as opposed to additive uncertainty).  
% The formula for USYS is 
%   USYS = (I + WT*ULTIDYN('Name',[size(P,1) size(P,1)]))*P
%
% [USYS,WT] = LTIARRAY2USS(P,PARRAY,ORD,'Additive') uses additive
% uncertainty.  This is the default uncertainty type. 
%   USYS = P + WT*ULTIDYN('Name',[size(P,1) size(P,2)])
%
% [USYS,WT] = LTIARRAY2USS(P,PARRAY,ORD,'InputMult') uses multiplicative
% uncertainty at the plant input (this is the default). The formula for USYS is 
%   USYS = P*(I + WT*ULTIDYN('Name',[size(P,2) size(P,2)]))
%
% [USYS,WT,DIFFDATA] = LTIARRAY2USS(P,PARRAY,ORD,TYPE) returns the norm of
% the difference (absolute difference for Additive, and relative difference
% for Multiplicative) between the nominal model P and PARRAY.  WT satisfies
% DIFFDATA(w_i) < |WT(w_i)| at all frequency points.
%
% See also: UCOVER

function [Usys,WT,pkdata] = ltiarray2uss(Pnom,Parray,order,type)

%   Copyright 2003-2009 The MathWorks, Inc.

if nargin<4
   type = [];
end
if isempty(type)
   type = 'InputMult';
end

[Usys,Info] = ucover(Parray,Pnom,order,[],type);
WT = Info.W1*Info.W2;
pkdata = Info.W1opt*Info.W2opt;
