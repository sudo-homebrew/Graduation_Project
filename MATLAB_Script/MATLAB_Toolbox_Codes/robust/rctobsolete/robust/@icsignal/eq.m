%EQ  Equate ICSIGNAL expressions
%   A==B is the ICSIGNAL object capturing the defined equality
%   of the ICSIGNAL objects A and B.  The Equation
%   property within ICONNECT objects is a cell array, whose
%   entries are set by equating ICSOGNAL objects to define
%   the relationship.
%
%   A or B may also be the zero-vector of appropriate dimension
%   (or even just the scalar 0) to equate an ICSIGNAL object to
%   zero.
%
%   See also ICONNECT, ICSIGNAL.

% Copyright 2004 The MathWorks, Inc.

function c = eq(a,b)
if isa(a,'icsignal') && isa(b,'icsignal')
   c = a-b;
elseif isa(a,'double') && ndims(a)==2 && min(size(a))==1 && max(abs(a))==0 && ...
      (size(b.System,1)==length(a) || length(a)==1)
   c = b;
elseif isa(b,'double') && ndims(b)==2 && min(size(b))==1 && max(abs(b))==0 && ...
      (size(a.System,1)==length(b) || length(b)==1)
   c = a;
else
   msg = ['Both arguments must be ICSIGNAL objects, or\none may be the zero-vector.'];
   error(sprintf(msg));
end
