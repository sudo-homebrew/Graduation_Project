function insertEmptyLineBeforeCopyight(outFile)
%Insert a line before the copyright.

fid = fopen(outFile);
fileData = char(fread(fid)');
fclose(fid);
fileContent = insertBefore(fileData,'% Copyright',newline);
fileContent = insertBefore(fileContent,'    %   Copyright',newline);
fid = fopen(outFile,'w');
fwrite(fid,fileContent);
fclose (fid);
end