function cellout = csize(out,arg2,nin,nout);
% Replicates behavior of Matlab SIZE output exactly for
% user-defined classes.  See use in @umat/size, @pmat/size,...


% Copyright 2003-2004 The MathWorks, Inc.

if nin==2
   if nout<=1
      if isa(arg2,'double') & isequal(size(arg2),[1 1]) & ...
         floor(arg2)==ceil(arg2) & arg2>0
         if arg2>length(out)
            cellout{1} = 1;
         else
            cellout{1} = out(arg2);
         end
      else
         error('Invalid 2nd argument in SIZE');
      end
   else
      error('Unknown command option');
   end
elseif nout>1
   cellout = {};
   if nout>=length(out)      
      for i=1:length(out)
         cellout{i} = out(i);
      end
      for i=length(out)+1:nout
         cellout{i} = 1;
      end
   else
      for i=1:nout-1
         cellout{i} = out(i);
      end
      cellout{nout} = prod(out(nout:end));
   end
elseif nout<=1
   cellout{1} = out;
end
