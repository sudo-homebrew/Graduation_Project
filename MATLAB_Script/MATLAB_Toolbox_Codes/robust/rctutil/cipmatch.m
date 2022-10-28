function [idx,fstr] = cipmatch(clist,str)
%

% Copyright 2003-2004 The MathWorks, Inc.

% case-insensitive partial match.
% if there are
% multiple matches, it looks for the exact,
% case-sensitive one.  Not yet.  Soon.

% .m
% .M
% .Matrix
% .Ma
%
% m -> 1
% M -> 2
% ma -> 4
% MA -> 4
% Ma -> 4
% Mat -> 3
% mat -> 3

lstr = length(str);
tf = find(strncmpi(clist,str,lstr));
if length(tf)==1
   idx = tf;
   fstr = clist{idx};
elseif length(tf)==0
   idx = [];
   fstr = [];
else
	cnt = 0;
   for i=1:length(tf)
   	if length(clist{tf(i)})==lstr
      	tfuse = tf(i);
      	cnt = cnt + 1;
      end
   end
   if cnt==1
      idx = tfuse;
      fstr = clist{idx};
   else
      tf = find(strncmp(clist,str,lstr));
      if length(tf)==1
         idx = tf;
         fstr = clist{idx};
      elseif length(tf)==0
         idx = [];
         fstr = [];
      else
	     cnt = 0;
         for i=1:length(tf)
         	if length(clist{tf(i)})==lstr
      	     tfuse = tf(i);
      	     cnt = cnt + 1;
            end
         end
         if cnt==1
            idx = tfuse;
            fstr = clist{idx};
         else
            error(['Ambiguous property: ''' str '''.']);
         end
      end
   end
end