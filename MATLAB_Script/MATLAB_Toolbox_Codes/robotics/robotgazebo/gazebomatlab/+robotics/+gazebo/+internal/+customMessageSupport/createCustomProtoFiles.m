function createCustomProtoFiles(protoList, ProtoFolderPath)
%This function is for internal use only. It may be removed in the future.
%
% This function reads all proto files ( custom messages ) and generates
% protobuf source files such as *.pb.cc and *.pb.h
% input :
%               @param protoList                    List of all proto file name and details
%               @param ProtoFolderPath              Output folder path to generate protobuf source files
%

%   Copyright 2019-2020 The MathWorks, Inc.

    %% clear previous generated files
    % remove all previous build headers and source of proto files
    headerFileNames = dir(fullfile(ProtoFolderPath,'*.pb.h'));
    sourceFileNames = dir(fullfile(ProtoFolderPath,'*.pb.cc'));
    protoFileNames = dir(fullfile(ProtoFolderPath,'*.proto'));

    for idx=1:length(headerFileNames)
        delete(fullfile(ProtoFolderPath, headerFileNames(idx).name));
    end
    for idx=1:length(sourceFileNames)
        delete(fullfile(ProtoFolderPath, sourceFileNames(idx).name));
    end
    for idx=1:length(protoFileNames)
        delete(fullfile(ProtoFolderPath, protoFileNames(idx).name));
    end

    %% read and generate protobuf source files for each .proto file
    % read all proto files and store as a string
    for i=1:size(protoList,1)

        filePath = fullfile(protoList(i).folder, protoList(i).name);

        % copy into proto folder
        copyfile(filePath,ProtoFolderPath);
    end

    % path of protoc .exe copied
    protoLibInstalledPath = fullfile(matlabroot, 'bin', computer('arch'));

    % create string to call protoc command
    protocPath = ['"',fullfile(protoLibInstalledPath,'protoc'),'" '];

    for i=1:size(protoList,1)
        % create command to generate proto headers and cpp files
        customProtoFile = [protocPath,' --proto_path="',ProtoFolderPath,'" --cpp_out=dllexport_decl=LIBMSGPROTO_EXPORT:"',...
                           ProtoFolderPath,'" "',fullfile(ProtoFolderPath,protoList(i).name),'" '];

        % generate cpp and headers
        % this will validate the proto files and provides warning
        system(customProtoFile);

        % retrieve proto file name
        fileName = erase(protoList(i).name,'.proto');

        % validate the structure of user defined proto files
        if (~(exist(fullfile(ProtoFolderPath,[fileName,'.pb.cc']),'file') && exist(fullfile(ProtoFolderPath,[fileName,'.pb.h']),'file')))
            error(message('robotics:robotgazebo:gazebogenmsg:InvalidProtobufFormat', fileName));
        end

    end

end
