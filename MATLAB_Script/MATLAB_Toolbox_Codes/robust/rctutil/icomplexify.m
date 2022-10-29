function deltar = icomplexify(deltacr)
%ICOMPLEXIFY  Helper function for COMPLEXIFY.
%   DELTAR = ICOMPLEXIFY(DELTACR) extracts a real value of the 
%   uncertainty blocks of DELTACR which were complexified via COMPLEXIFY.
%
%   ICOMPLEXIFY only affects field pairs of DELTACR named 'foo' and 
%   'foo_cmpxfy' where 'foo' can be any field name.   DELTAR is the same
%   as DELTACR except that the fields 'foo_cmpxfy' are removed.
%   COMPLEXIFY, by default, complexifies the real uncertainty with 
%   UCOMPLEX atoms, though optionally ULTIDYN atoms can be used.  If
%   an UCOMPLEX uncertainty was used to complexify the uncertain system,  
%   the real parts of 'foo_cmpxfy' are added to the real parts of 'foo'. 
%   If an ULTIDYN uncertainty  was used to complexify the uncertain system,  
%   only the real parts of 'foo' are returned. 
%
%   See also COMPLEXIFY, ROBSTAB.


%   Copyright 2006 The MathWorks, Inc.

fn = fieldnames(deltacr);
szD = size(deltacr);

deltar = deltacr;

for i = 1:length(fn);
   fname = fn{i};
   if length(fname)>=8 && strcmp(fname(end-6:end),'_cmpxfy')
      idx = find(strcmp(fname(1:end-7),fn));
      if length(idx)==1
         for  j=1:prod(szD)
            realpart = getfield(deltacr,{j},fname(1:end-7),{1});
            cmplxpart = getfield(deltacr,{j},fname,{1});
            if isa(cmplxpart,'double')
               val = realpart + real(cmplxpart);
            else
               val = realpart;
            end
            deltar = setfield(deltar,{j},fname(1:end-7),{1},val);
         end
         deltar = rmfield(deltar,fname);
      end
   end
end