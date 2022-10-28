function statusCode = hoopsConvert(infile, outfile)
%This function is for internal use only. It may be removed in the future.

%HOOPSCONVERT Convert DAE CAD files to STL format
%   HOOPSCONVERT converts the DAE file defined by INFILE to STL file
%   defined by OUTFILE. The STL file is written to the path defined by
%   OUTFILE.

%   Copyright 2021 The MathWorks, Inc.

    hoopsWrappper = robotics.utils.internal.HoopsExchangeWrapper.getInstance;

    %Perform conversion
    statusCode = hoopsWrappper.hoopsConvert(infile, outfile);
end
