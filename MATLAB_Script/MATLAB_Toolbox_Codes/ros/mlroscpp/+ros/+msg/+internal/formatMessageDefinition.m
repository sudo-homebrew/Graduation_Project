function mlMsgDef = formatMessageDefinition(rosMsgDef, rosPropList, propList)
%This function is for internal use only. It may be removed in the future.

%FORMATMESSAGEDEFINITION Format ROS message definition for MATLAB display
%   This function adjusts all property names, so they conform to MATLAB
%   naming standards. All comments and whitespaces are also properly formatted.
%   ROSPROPLIST is the list of property names in the ROS message
%   definition and PROPLIST are the corresponding property names in MATLAB.
%   Both ROSPROPLIST and PROPLIST should be cell arrays of strings of the
%   same size. If you don't want to replace property names, specify both
%   inputs as empty cell arrays, {}.

%   Copyright 2019 The MathWorks, Inc.

msgDef = rosMsgDef;

% Use regular expression to replace property names
for i = 1:numel(rosPropList)
    msgDef = regexpSelective(msgDef, rosPropList{i}, propList{i});
end

% Make sure to replace all constants, since they are not part of the
% message's property list. Since it's less likely that other text
% elements would start with a series of uppercase letters, followed by
% an underscore and followed by another series of uppercase letters, we
% can make this replacement more lenient (not referenced to the
% beginning of a line).
msgDef = regexprep(msgDef, '([A-Z]+)[_]([A-Z]*[\s\n#])', '$1$2');

% Convert ROS message types to MATLAB data types
% Replace all deprecated data types
msgDef = regexpBeginningOfLine(msgDef, 'char', 'uint8');
msgDef = regexpBeginningOfLine(msgDef, 'byte', 'int8');

% Replace all common primitive data types
msgDef = regexprep(msgDef, 'float32', 'single');
msgDef = regexprep(msgDef, 'float64', 'double');
msgDef = regexpBeginningOfLine(msgDef, 'bool', 'logical');
msgDef = regexpBeginningOfLine(msgDef, 'string', 'char');
msgDef = regexpSelective(msgDef, 'time', 'Time');
msgDef = regexpSelective(msgDef, 'duration', 'Duration');
msgDef = regexpBeginningOfLine(msgDef, 'Header', 'std_msgs/Header');

% Align all comments that don't start in the first column
% First, find the maximum column of all comments
strLines = strsplit(msgDef, '\n')';
strfind(strLines, '#');
commentLocation = cell2mat(regexp(strLines, '(?m)^[^#]+#', 'end'));
maxLoc = max(commentLocation);

% Second, align all comments with the maximum column
if ~isempty(maxLoc)
    msgDef = regexprep(msgDef, '(?m)^([^#\n]+)#', '$1${repmat('' '', 1, (maxLoc-length($1)-1))}%');
end

% Replace all remaining comment characters
mlMsgDef = regexprep(msgDef, '#', '%');

end

function newString = regexpBeginningOfLine(oldString, expression, replace)
%regexpBeginningOfLine Match and replace a string at the beginning of line
%   This also ignores any whitespace characters that might be in front of
%   the expression.
%   The token $1 ensures that all the white spaces that were matched before
%   EXPRESSION are preserved in the replacement string.
newString = regexprep(oldString, ['(?m)(^\s*)' expression], ['$1' replace]);
end

function newString = regexpSelective(oldString, expression, replace)
%regexpSelective Match and replace a string more liberally
%   As long as the string is surrounded by white spaces or some other
%   calculation signs, or quotation marks, this match will work.
newString = regexprep(oldString, ['(?m)([-#\(+*/"\s]|^)' expression '([-.,:\)=+*/"#\n\s]|$)'], ['$1' replace '$2']);
end
