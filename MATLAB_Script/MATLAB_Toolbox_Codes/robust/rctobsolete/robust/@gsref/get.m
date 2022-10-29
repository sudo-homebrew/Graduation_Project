function out = get(m,property)
%GET Get properties of object

%   Copyright 2003-2011 The MathWorks, Inc.

GSp = pvget(m,'PropNames');
if nargin==2
   % Use CaseInsensitivePartialMatch to find the correct property name
   [idx,fstr] = cipmatch(GSp.GPropNames,property);
   if isempty(idx)
      error(['Property ' property ' is not gettable']);
   else
      % Extract value with PVGET
      [out,pflag,errstring] = pvget(m,fstr);
      if pflag==0
         error(errstring)
      end
   end
else
   % Create an empty STRUCT, with all the entries from GProp as
   % fieldsnames
   nprop = length(GSp.GPropNames);
   plist = [GSp.GPropNames cell(nprop,1)]';
   if isempty(plist)
      tout = struct([]);
   else
      tout = struct(plist{:});
   end
   % Fill each field with the appropriate value
   for i=1:nprop
      tout.(GSp.GPropNames{i}) = pvget(m,GSp.GPropNames{i});
   end
   if nargout==1
      out = tout;
   else
      % If there are no output arguments, simply display the result 
      disp(tout);
   end
end
