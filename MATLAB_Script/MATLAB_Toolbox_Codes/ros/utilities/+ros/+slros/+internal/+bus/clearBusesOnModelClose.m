function clearBusesOnModelClose(block)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2018-2021 The MathWorks, Inc.

%clearBusesOnModelClose Clear buses from global scope on model close


if ~ros.slros.internal.block.CommonMessageMask.isLibraryBlock(block)
    [~,fileName,fileExt] = fileparts(ros.slros.internal.bus.Util.ROSDataDict);
    Simulink.data.dictionary.closeAll([fileName,fileExt],'-discard')
end

end
