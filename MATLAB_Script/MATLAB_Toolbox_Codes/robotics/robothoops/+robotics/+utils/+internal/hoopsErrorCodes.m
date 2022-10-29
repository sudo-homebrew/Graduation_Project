classdef hoopsErrorCodes < double
    %This class is for internal use only. It may be removed in the future.

    %hoopsErrorCodes HOOPS Exchange error code enumeration.

    %   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        HOOPS_WRITE_ERROR(-2)
        HOOPS_WRITER_NOT_IMPLEMENTED(-3)
        HOOPS_LOAD_INVALID_FILE_FORMAT(-4)
    end
end