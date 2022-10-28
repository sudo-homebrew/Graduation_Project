function isNewCode = hasModelChanged
%This function is for internal use only. It may be removed in the future.

%hasModelChanged Verify if model has changed
%   This function should only be called in the model build context. If the
%   function is called in any other context, it will always indicate that
%   the model has changed.

%   Copyright 2016-2018 The MathWorks, Inc.

isNewCode = codertarget.tools.AfterCodeGenGlobals.getIsNewGeneratedCode;
