% Copyright 2003-2004 The MathWorks, Inc.

function Match=keymatch(keyi,DocKeys,UndocKeys)
% function robust/private/keymatch.m
%
% MATCH=KEYMATCH(KEY,DOCKEYS,UNDOCKEYS)
%  Completes string KEY by looking for a match in the 
%  cell arrays of string DOCKEYS. The optional input UNDOCKEYS,
%  if present, is also searched.  
% 
%  An error rsults if either no match is foundor the match is not unique.  
%  In case of no match, the error message suggests choosing one of the
%  DocKeys. In the case of multiple matches, the error tells the user
%  that the give KEY is ambiguous and lists the possible matches from
%  in the cell array [DOCKEYS, UNDOCKEYS].
%
%  IMPORTANT
%  Each element of [DOCKEYS, UNDOCKEYS] must end with a single space
%  of the error messages will not display properly.
%
%  Example, from lti/hinfsyn
%       DocKeys={'GMAX ','GMIN ','TOLGAM ','METHOD ','S0 ','VERBOSE '};
%       UndocKeys={'EPR ','EPP ','SO ','LMIOPTIONS ','OPTIONS '};

% if KEYI is a cell array containing a single string, overwrite
% KEYI with the string
% if iscell(keyi) & isequal(length(keyi),1))
%     keyi=keyi{i};
% else
%     error('Key is not a string')
% end

if ~isstr(keyi) | ~isequal(size(keyi,1),1)
    error(['KEY is not a string'])
end
if nargin<2, 
    UndocKeys=[];
end

AllKeys=horzcat(DocKeys,UndocKeys);
ikey=strmatch(upper(keyi),AllKeys);
switch length(ikey)
    case 0 
        display(['Key ''' keyi ''' is invalid.'])
        error(['Use one of the following: ' horzcat(DocKeys{:})])
    case 1
        Match=upper(AllKeys{ikey});
    otherwise
        display(['Key ''' keyi ''' is ambiguous.'])
        error(['Matches all of the following: ' horzcat(AllKeys{ikey})])
end
%%% end robust/private/keymatch.m
