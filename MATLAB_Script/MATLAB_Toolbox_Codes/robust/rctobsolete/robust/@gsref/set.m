%SET Set GSREF object properties

function pout = set(p,varargin)
%   Copyright 2003-2011 The MathWorks, Inc.

nin = nargin;
ninp = (nin-1)/2;
classp = upper(class(p));
GSp = pvget(p,'PropNames');
if nin>=2
   if floor(ninp)==ceil(ninp)
      tpout = p;
      for i=1:ninp
         try
            [idx,~] = cipmatch(GSp.SPropNames,varargin{2*i-1});
         catch ME
            throw(ME)
         end
         if ~isempty(idx)
            if isequal(GSp.SPropNames{idx},'dynamicsys')
               [tpout] = pvset(tpout,GSp.SPropNames{idx},varargin{2*i});
            else
               [tpout,pflag,errstring] = pvset(tpout,GSp.SPropNames{idx},varargin{2*i});
               if pflag==0
                  error(errstring)
               end
            end
         else
             % Property is not in the settable property list, so 
             % this will error out.  Try to provide specific error type.
            try
               [idx,~] = cipmatch(GSp.GPropNames,varargin{2*i-1});
               if ~isempty(idx)
                  error(['Property ' GSp.GPropNames{idx} ' is read only']);
               else
                  error(['Property ' varargin{2*i-1} ' is not gettable.']);
               end
            catch ME
               throw(ME)
            end
         end
      end
      [pflag] = isvalid(tpout);
      if pflag==1
         if nargout==1
            pout = tpout;
         else
            if isempty(inputname(1))
               error('First argument in SET is an expression. Needs to be a variable.')
            else
               assignin('caller',inputname(1),tpout);
            end
         end
      else
         error(['Invalid ' classp ' construction']);
      end
   else
      error('Invalid Property/Value list');
   end
else
   % Create an empty STRUCT, with all the entries from SProp as
   % fieldsnames
   plist = [GSp.SPropNames GSp.SPropDescription]';
   if isempty(plist)
      tout = struct([]);
   else
      tout = struct(plist{:});
   end
   if nargout==1
      pout = tout;
   else
      % If there are no output arguments, simply display the result 
      disp(tout);
   end
end
