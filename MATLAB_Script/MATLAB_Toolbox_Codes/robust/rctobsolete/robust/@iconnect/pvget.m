function [out,pflag,errstring] = pvget(m,property)
%

% Copyright 2003-2004 The MathWorks, Inc.

pflag = 1;
errstring = '';
switch property
case {'Equation'}
   out = m.Equation;
case {'Input'}
   out = m.Input;
case {'Output'}
   out = m.Output;
case {'System'}
      nu = size(m.Input,1);
      ny = size(m.Output,1);
      if nu==0 || ny==0
         out = [];
      else
         ActualU = icsignal(nu);
         ActualY = icsignal(ny);
         Ex1 = m.Input == ActualU;
         Ex2 = m.Output == ActualY;

         AllC = [];
         for i=1:length(m.Equation)
            AllC = [AllC;m.Equation{i}];
         end
         nc = size(AllC,1);
         AllC = [AllC;m.Input;m.Output];
         ns = size(pvget(AllC,'System'),2);

         Sys = [pvget(AllC,'System') [zeros(nc,nu+ny);-eye(nu+ny)]];
         UIDX = ns+(1:nu);
         YIDX = [1:ns ns+nu+(1:ny)];
         out = imp2exp(Sys,YIDX,UIDX);
         out = out(ns+(1:ny),:);
      end
case 'PropNames'
   out.GPropNames = {'Equation';'Input';'Output';'System'};
   out.SPropNames = {'Equation';'Input';'Output'};
   out.SPropDescription = {'Cell array of Equality Constraints';...
         'ICSIGNAL';'ICSIGNAL'};
otherwise
   out = [];
   pflag = 0;
   errstring = ['Invalid ICONNECT property: ''' property '''.'];
end
if nargout<=1 && pflag==0
   error(errstring);
end
