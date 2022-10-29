function blk = blkstruct2N2(bs,allowsubs)

%   Copyright 2004-2011 The MathWorks, Inc.

if nargin==1
   allowsubs = 0;
end

if isa(bs,'struct') && ismatrix(bs) && size(bs,2)==1
   blk = zeros(0,2);
   for i=1:length(bs)
      if bs(i).Occurrences>0
         switch bs(i).Type
            case {'ureal'}
               blk = [blk;-bs(i).Occurrences 0]; %#ok<*AGROW>
            case {'ucomplex'}
               blk = [blk;bs(i).Occurrences 0];
            case {'ultidyn' 'ucomplexm' 'udyn' 'umargin'}
               if bs(i).Occurrences==1 || isequal(bs(i).Size,[1 1])
                  if bs(i).Occurrences==1
                     blk = [blk;bs(i).Size];
                  else
                     blk = [blk;bs(i).Occurrences 0];
                  end
               elseif allowsubs
                  % repeated, non-scalar, make them all separate elements.
                  % upper bound will be conservative, but still an upper
                  % bound.
                  blk = [blk;repmat([bs(i).Size],[bs(i).Occurrences 1])];
               else
                  msg1 = ['Uncertain element ' bs(i).Name ' is a '];
                  msg2 = ['repeated, non-scalar ' upper(bs(i).Type) '.\n'];
                  msg3 = 'MUSSV cannot be used in this case.';
                  msg = sprintf([msg1 msg2 msg3]);
                  error(msg); %#ok<SPERR>
               end
            otherwise
               error('Unrecognized Uncertain Atom');
         end
      end
   end
elseif isa(bs,'double') && ismatrix(bs) && size(bs,2)==2
   blk = bs;
   fz = sum(abs(blk),2);
   locz = find(fz==0);
   if ~isempty(locz)
      blk(locz,:) = [];
   end
elseif isempty(bs)
   blk = zeros(0,2);
else
   error('Invalid Block Structure.')   
end
            
            
