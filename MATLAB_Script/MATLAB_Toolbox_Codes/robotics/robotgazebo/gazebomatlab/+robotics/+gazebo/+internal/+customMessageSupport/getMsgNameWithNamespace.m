function [messageNameWithNamespace, Namespace] = getMsgNameWithNamespace(messageNameSpace,messageName,standard)
%This function is for internal use only. It may be removed in the future.
%
% getMsgNameWithNamespace function returns message name and namespace based of
% input namespace and message name. The output message name and namespace has
% C++ or Simulink message format
%
% e.g. Proto file contains
% input :
%    package msg;
%    message A
%    {
%    }
% output:
%    messageNameWithNamespace = 'msg_A'  ( Simulink ) and 'msg::A' ( C++ )
%    Namespace = 'msg::' ( Simulink and C++ )

%   Copyright 2020 The MathWorks, Inc.

    switch(standard)
      case 'Simulink'
        if(~isempty(messageNameSpace))
            Namespace = [char(messageNameSpace),'::'];
            messageNameWithNamespace = [char(strrep(messageNameSpace,'::','_')),'_',char(messageName)];
        else
            Namespace = '';
            messageNameWithNamespace = char(messageName);
        end

      case 'C++'
        if(~isempty(messageNameSpace))
            Namespace = [char(messageNameSpace),'::'];
            messageNameWithNamespace = join([string(messageNameSpace),'::',messageName],'');
        else
            Namespace = '';
            messageNameWithNamespace = char(messageName);
        end
    end

end
