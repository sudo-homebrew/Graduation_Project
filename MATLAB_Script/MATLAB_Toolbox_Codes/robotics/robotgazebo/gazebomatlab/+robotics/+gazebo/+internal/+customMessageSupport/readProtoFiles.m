function [protoFileStruct] = readProtoFiles(protoFolderPath, protoFileName)
%This function is for internal use only. It may be removed in the future.
%
% This function reads single proto file and recognizes fields
% defined by the user. If single proto files contains multiple messages
% then this code separates messages. Further, fields like message names,
% datatypes, variable format( required, repeated or optional) of each message
% is separated and store into MATLAB structure.

%   Copyright 2019-2020 The MathWorks, Inc.

% input :
%               @param protoFolderPath                   Folder path of .proto file present
%               @param protoFileName                 Single .proto file name
%
% output:
%               @param protoFileStruct               MATLAB struct containing proto message details
%

    %% read Proto file C++ API
    gazeboCustomParser = robotics.internal.GazeboClient;

    % validate unsupported proto fields
    status = gazeboCustomParser.validateMessageFields(protoFolderPath,protoFileName);

    if(status)
        error(message('robotics:robotgazebo:gazebogenmsg:UnsupportedKeyword', protoFileName));
    end

    % get MATLAB struct from proto file
    protoFileStruct = gazeboCustomParser.getProtoStruct(protoFolderPath,protoFileName);

end
