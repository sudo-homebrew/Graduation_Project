%VERTCAT  Vertical concatenation of ICSIGNAL objects.
%  
%ICS = VERTCAT(ICS1,ICS2,...) performs a concatenation operation of 
%  ICS = [ICS1; ICS2; ...].
%
%See also: HORZCAT

% Copyright 2003-2004 The MathWorks, Inc.
function out = vertcat(varargin)

if nargin==1;
   out = varargin{1};
elseif nargin==2
   A = varargin{1};
   B = varargin{2};
   if isempty(A)
      out = B;
   elseif isempty(B)
      out = A;
   else
      [a2n,b2n,listn,dimn] = ...
         icsigbin(A.SignalList,A.SignalDim,B.SignalList,B.SignalDim);
      sysout = [A.System*a2n;B.System*b2n];
      out = icsignal(sysout,listn,dimn);
   end
else
    out = vertcat([varargin{1};varargin{2}],varargin{3:end});
end
