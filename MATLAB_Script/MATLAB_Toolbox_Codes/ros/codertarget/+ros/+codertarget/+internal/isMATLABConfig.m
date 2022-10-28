function ret = isMATLABConfig(hCS)
%This function is for internal use only. It may be removed in the future.
%
%ISMATLABCONFIG Return true if input is a MATLAB codegen configuration object

% Copyright 2020 The MathWorks, Inc.
ret = isa(hCS,'coder.EmbeddedCodeConfig') || isa(hCS,'coder.CodeConfig');
end

