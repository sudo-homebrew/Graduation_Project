function isValid = isValidMessageName(msgName)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%checks the validity of the msgName 
%file names must use an upper camel case name and only consist of alphanumeric characters.
%http://design.ros2.org/articles/interface_definition.html#naming-of-messages-and-services
matchedName = regexp(msgName,'^[A-Z][A-Za-z0-9]*$','match');
isValid = ~isempty(matchedName) && isequal(msgName, matchedName{1});