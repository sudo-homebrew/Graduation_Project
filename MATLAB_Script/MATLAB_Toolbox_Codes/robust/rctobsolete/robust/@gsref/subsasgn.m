function m = subsasgn(m,L,RHS)
%SUBSASGN  Subscripted assignment for GSREF objects.

%   Copyright 2003-2011 The MathWorks, Inc.

% XXX needs to be checked carefully.
% Note: if A<B, and prog.m in method(A), but prog.m not_in method(B),
%  and if m1 in A, m2 in B, then prog(A,B) errors out, as the @A/prog is
%  not found.
try
   switch L(1).type
      case '.'
         if length(L)==1
            tmp = RHS;
         else
            tmp = subsref(m,L(1));
            tmp = subsasgn(tmp,L(2:end),RHS);
         end
         m = set(m,L(1).subs,tmp);
      case '()'
         if length(L)==1
            tmp = RHS;
         else
            tmp = subsref(m,L(1));
            tmp = subsasgn(tmp,L(2:end),RHS);
         end
         m = parenset(m,L(1).subs,tmp);
      otherwise
         if iscell(m) && isa(RHS,'gsref')
            if length(L)==1
               tmp = RHS;
            else
               switch flag
                  case 1
                     tmp = builtin('subsref',m,L(1));
                  otherwise
                     tmp = subsref(m,L(1));
               end
               tmp = subsasgn(tmp,L(2:end),RHS);
            end
            m = builtin('subsasgn',m,L(1),tmp);
         else
            error(['Cell-like SUBSASGN is not supported for ' class(m) ' objects.']);
         end
   end
catch ME
   throw(ME)
end
