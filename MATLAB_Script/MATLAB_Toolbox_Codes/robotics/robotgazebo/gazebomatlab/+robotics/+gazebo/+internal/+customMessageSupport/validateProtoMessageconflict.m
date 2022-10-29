function validateProtoMessageconflict(customDetails)
%This function is for internal use only. It may be removed in the future.
%
% This function validate the message names defined by user. If the message
% name is defined in same or multiple '.proto' files then it error-out.
%
% input :
%       @param customDetails             The custom message definition details
%
% Case A : protoc throw error in following conditions,
%    1] If 'message A'  name used multiple times in single A.proto file
%    2] If 'message A'  name is used in A.proto file and B.proto and B.proto is
%       imported in A.proto
%
% Case B: protoc don't throw error in following conditions,
%    1] If 'message A'  name is used  in A.proto file and B.proto and there is no
%       link(import) between A.proto and B.proto
%
% Case C: Error in internal protobuf messages
%    Protobuf don't allow to use same message name and namespace multiple time
%    Few Protobuf messages are already used for internal purpose.
%    Thus, objective is to stop user for using same namespace for custom message
%
% This function targets Case B and Case C.

%   Copyright 2019-2020 The MathWorks, Inc.

    %% handling 'Case C'
    % check message namespace defined in the '.proto' file conflicts with internal used message namespace

    % internal message name and namespace list
    internalMessage_NameSpace = 'mw::internal::robotics::gazebotransport'; % stored in extraction process ( C++ standard )
    internalMessageNameSpace = 'mw.internal.robotics.gazebotransport';   % defined in '.proto' file

    for idx = 1:length(customDetails.messageNameList)
        conflictNameSpace = char(customDetails.nameSpace{idx});
        % verify message namespace
        if(strcmp(internalMessage_NameSpace,conflictNameSpace))
            protoFileName = customDetails.protoFileName{idx};
            error(message('robotics:robotgazebo:gazebogenmsg:InternalMessageNameConflicts', internalMessageNameSpace,protoFileName));
        end

    end

    %% handling 'Case B'

    % check a message name defined in multiple '.proto' file
    % I. Two or more .proto files are valid if
    %    a] Different message name in all files
    %        b] Same message name but different namespace
    %        c] Same message name, same namespace but one message name is parent and
    %               other one is child or base
    %
    % II. Two or more .proto files are invalid if
    %        a] Same message name with no namespace in all files.
    %    b] Same message name with same namespace in all files.


    for idx = 1:length(customDetails.messageNameList)
        % verify message name is present in all message list
        % retrieve index of same message name location
        repId = find(strcmp(customDetails.messageNameList{idx},customDetails.messageNameList));

        % length of index (repId) is greater than 1 means same message name is
        % present in multiple files
        if(length(repId) > 1)
            conflictName = customDetails.messageNameList{idx};

            % checks conflict message names are parent or base name
            % retrieve index of conflict message names parent fields
            conflictParentNamesId = repId(cellfun(@isempty,customDetails.parentMessageNameList(repId)));

            % Conflict message name should not base name
            % length(conflictParentNamesId) > 1 means conflict message names are Parent ( validate case I.c] )
            % isempty(customDetails.parentMessageNameList{idx}) means message name is Parent ( validate case I.c] )
            if( length(conflictParentNamesId) > 1 && isempty(customDetails.parentMessageNameList{idx}))
                % retrieve conflict message namespace
                conflictNameSpace = [customDetails.nameSpace{conflictParentNamesId}];
                % verify message namespace is conflict too
                if( isempty(customDetails.nameSpace{idx}) && isempty(conflictNameSpace))
                    % both empty means same namespace
                    % case II.a]
                    protoFileName = join(string(customDetails.protoFileName(conflictParentNamesId)),' ');
                    error(message('robotics:robotgazebo:gazebogenmsg:MessageNameConflicts', conflictName,protoFileName));
                else
                    % retrieve same namespace index
                    spaceId = find(cellfun(@(x)isequal(x,customDetails.nameSpace{idx}),customDetails.nameSpace));
                    % find common index of conflicts parent messages and same namespace
                    intersecId = intersect(spaceId,conflictParentNamesId);
                    if( length(intersecId) > 1 )
                        % means conflict message name has same message namespace
                        % case II.b]
                        conflictNameWithNameSpace = [char(customDetails.nameSpace{idx}),'::',conflictName];
                        protoFileName = join(string(customDetails.protoFileName(intersecId)),' ');
                        error(message('robotics:robotgazebo:gazebogenmsg:MessageNameConflicts', conflictNameWithNameSpace,protoFileName));
                    end
                end
            end

        end
    end

end
