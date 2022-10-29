function locAddNode(name)
%LOCADDNODE Add Name to the list of nodes.

%   Copyright 2021 The MathWorks, Inc.
cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
addNode(cgenInfo,name);
end
