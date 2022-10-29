function ret = convertLowercaseUnderscoreToMixedCase(val)
%This function is for internal use only. It may be removed in the future.

%CONVERTLOWERCASEUNDERSCORETOMIXEDCASE returns a string after converting a lowercase string to mixed case by removing underscore if required.

%   Copyright 2020-2021 The MathWorks, Inc.

% Look for the first lowercase letter in the name or first after underscore
% Numbers, uppercase letters, and underscores don't need capitalization
    expression = '(^|_)[a-z]';

    % Use dynamic expression to uppercase the letter
    replace = '${upper($0)}';

    % Perform replacement with capital letters
    val = regexprep(val,expression,replace);

    % Remove all underscores
    ret = regexprep(val,'_','');
end
