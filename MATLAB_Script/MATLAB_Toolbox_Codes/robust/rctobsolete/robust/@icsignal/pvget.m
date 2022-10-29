%PVGET  Get values of public ICSIGNAL object properties.
%  
%VALUES = PVGET(MAT) returns all public values in a cell
%array VALUES.
%  
%VALUE = PVGET(SYS,PROPERTY) returns the value of the
%single property with name PROPERTY.
%  
%See also GET.

% Copyright 2003-2004 The MathWorks, Inc.
function [out,pflag,errstring] = pvget(m,property)

pflag = 1;
errstring = '';
switch property
case 'System'
   out = m.System;
case 'SignalList'
   out = m.SignalList;
case 'SignalDim'
   out = m.SignalDim;
case 'PropNames'
   out.GPropNames = cell(0,1);
   out.SPropNames = cell(0,1);
   out.SPropDescription = cell(0,1);
otherwise
   out = [];
   pflag = 0;
   errstring = ['Invalid ICSIGNAL property: ''' property '''.'];
end
if nargout<=1 && pflag==0
   error(errstring);
end
