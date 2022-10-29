function m = subsref(m,L)
%SUBSREF  Subscripted reference for GSREF objects.

%   Copyright 2003-2011 The MathWorks, Inc.
try
   switch L(1).type
      case '.'
         m = get(m,L(1).subs);
      case '()'
         m = parenget(m,L(1).subs);
      case '{}'
         error(['{}- like reference is not supported for ' upper(class(m)) ' objects.']);
   end
   if length(L)>1
      m = subsref(m,L(2:end));
   end
catch ME
   throw(ME)
end
