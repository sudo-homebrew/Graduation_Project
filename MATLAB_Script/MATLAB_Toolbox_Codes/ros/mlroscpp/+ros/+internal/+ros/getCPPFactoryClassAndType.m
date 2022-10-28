function [cppFactoryClass , cppElementType] = getCPPFactoryClassAndType(msgType)
    % check if the message is part of any service/action or not.

    %   Copyright 2020-2021 The MathWorks, Inc.

    [pkgName, msgName] = fileparts(msgType);

    if endsWith(msgName,'Request') && ...
            ismember(msgType(1:end-7),rostype.getServiceList)
       cppFactoryClass = [pkgName '_' msgName(1:end-7) '_service'];
       cppElementType = 'Request';
    elseif endsWith(msgName,'Response') && ...
            ismember(msgType(1:end-8),rostype.getServiceList)
       cppFactoryClass = [pkgName '_' msgName(1:end-8) '_service'];
       cppElementType = 'Response';
    elseif endsWith(msgName,'Goal') && ...
            ismember(msgType(1:end-4),rostype.getActionList)
       cppFactoryClass = [pkgName '_' msgName(1:end-4) '_action'];
       cppElementType = 'Goal'; 
    elseif endsWith(msgName,'Feedback') && ...
            ismember(msgType(1:end-8),rostype.getActionList)
       cppFactoryClass = [pkgName '_' msgName(1:end-length('Feedback')) '_action'];
       cppElementType = 'Feedback';
    elseif endsWith(msgName,'Result') && ...
            ismember(msgType(1:end-6),rostype.getActionList)
       cppFactoryClass = [pkgName '_' msgName(1:end-6) '_action'];
       cppElementType = 'Result';
    else 
       cppFactoryClass = [pkgName '_' msgName '_message'];
       cppElementType = 'Message';
    end    
end
