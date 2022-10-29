function validateMsg(pkgpath)
%This function is for internal use only. It may be removed in the future.

%validateMsg uses runros2py and validateMsg.py to validate messages under
%pkgpath

%   Copyright 2019 The MathWorks, Inc.

validateMsgPyPath = fullfile(fileparts(mfilename('fullpath')),'validateMsg.py');
cmd = ['"' validateMsgPyPath, '" "', pkgpath, '"'];
[stat, res] = ros.ros2.internal.runros2py(cmd);
if stat ~= 0
    parserEx = MException(message('ros:utilities:custommsg:ParserValidationError', ...
                                  strrep(res,'\','/')));
    ex = MException(message('ros:utilities:custommsg:InvalidCustomMessagesError'));
    throw(ex.addCause(parserEx))
end
