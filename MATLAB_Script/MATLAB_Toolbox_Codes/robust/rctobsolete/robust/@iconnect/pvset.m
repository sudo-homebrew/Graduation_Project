function [m,pflag,errstring] = pvset(m,property,value)
%

% Copyright 2003-2004 The MathWorks, Inc.

pflag = 1;
errstring = '';
switch property
case {'Equation'}
   szv = size(value);
   if isa(value,'cell') & ndims(value)==2 & (szv(1)==1 | szv(2)==1)
      pflag = 1;
      for i=1:length(value)
         if ~isa(value{i},'icsignal')
            pflag = 0;
            errstring = 'Equations must be ICSIGNAL class.';
         end
      end
      if pflag==1
         m.Equation = value;
      elseif nargout<=1
         pflag = 0;
         errstring = 'Invalid ''Equation'' cell array';
      end
   else
      pflag = 0;
      errstring = 'Invalid ''Equation'' cell array';
   end  
case {'Input'}
   if isa(value,'icsignal')
      m.Input = value;
   else
      pflag = 0;
      errstring = ['Invalid ICONNECT property ' property ': should be ICSIGNAL'];
   end
case {'Output'}
   if isa(value,'icsignal')
      m.Output = value;
   else
      pflag = 0;
      errstring = ['Invalid ICONNECT property ' property ': should be ICSIGNAL'];
   end
otherwise
   pflag = 0;
   errstring = ['Invalid ICONNECT property: ' property];
end
if nargout<=1 && pflag==0
   error(errstring);
end
   
