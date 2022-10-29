function value = convertCamelcaseToLowercaseUnderscore(string)
%This function is for internal use only. It may be removed in the future.

%CONVERTCAMELCASETOLOWERCASEUNDERSCORE returns a string after converting a
%   camelcase string to lowercase string and place an underscore in between,
%   if required.

%   Copyright 2019 The MathWorks, Inc.

    value = regexprep(string, '(.)([A-Z][a-z]+)', '$1_$2');
    value = regexprep(value, '([a-z0-9])([A-Z])', '$1_$2');
    value = lower(value);

end
