function [isValid,errMsgFieldsString] = isValidFieldName(msgFieldNames)
%This function is for internal use only. It may be removed in the future.

%isValidFieldName checks the validity of the field-names. Filed-name 
% cannot be a C++ keyword and in this function it is verified that 
% the field name is a C++ keyword or not. 
% It receives the list of the field-names and returns the list of the 
% invalid field-names as a comma separated string.
% 
% Example:
%     [isValid,errMsgFieldsString] = ros.internal.isValidFieldName({'int',...
%                                               'valid_name','namespace'})
%   Copyright 2020 The MathWorks, Inc.

cppKeyWords = {'alignas', 'alignof', 'and', 'and_eq', 'asm', 'atomic_cancel',...
        'atomic_commit', 'atomic_noexcept', 'auto', 'bitand', 'bitor', 'bool', 'break', ...
        'case', 'catch', 'char', 'char16_t', 'char32_t', 'class', 'compl',...
        'const', 'constexpr', 'const_cast', 'continue', 'decltype', 'default', 'delete', ...
        'do', 'double', 'dynamic_cast', 'else', 'enum', 'explicit', 'export',...
        'extern', 'false', 'float', 'for', 'friend', 'goto', 'if', 'inline', 'int', 'long', ...
        'mutable', 'namespace', 'new', 'not', 'not_eq', 'nullptr', 'operator',...
        'or', 'or_eq', 'private', 'protected', 'public', 'register', 'reinterpret_cast', ...
        'return', 'short', 'signed', 'sizeof', 'static', 'static_assert', ...
        'static_cast', 'struct', 'switch', 'synchronized', 'template', 'this', 'thread_local', ...
        'throw', 'true', 'try', 'typedef', 'typeid', 'typename', 'union', ...
        'unsigned', 'using', 'virtual', 'void', 'volatile', 'wchar_t', 'while', 'xor', 'xor_eq'
        };
 
whichAreKeywords = ismember(cppKeyWords,msgFieldNames);
errMsgFieldsString = '';
isValid = true;

if any(whichAreKeywords)
    isValid = false;
    errMsgFields = cppKeyWords(whichAreKeywords);
    errMsgFieldsString = strjoin(errMsgFields,', ');
end
    