function createFile(messageString,fileName)
%This function is for internal use only. It may be removed in the future.
%
%This function creates a txt file based on the file name and fills with
%the messageString
%

%   Copyright 2019 The MathWorks, Inc.
%
% input :
%               @param messageString         string which contains file contents
%               @param fileName                          name of created file


    fileID = fopen(fileName, 'wt');
    cleanup = onCleanup(@()fclose(fileID));
    fwrite(fileID, messageString);


end
