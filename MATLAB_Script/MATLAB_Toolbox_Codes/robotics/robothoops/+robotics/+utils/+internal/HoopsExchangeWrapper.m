classdef HoopsExchangeWrapper
%HoopsExchangeWrapper Wrapper to handle single instance of the HoopsExchange MCOS class.
%   Singleton wrapper class implementation to store a single instance of
%   the HoopsExchange MCOS class. hoopsConvert method converts files
%   from one format to another.

%   Copyright 2021 The MathWorks, Inc.

    properties(Access=private)
        % Placeholder for HoopsExchange MCOS class
        HoopsExchangeMCOS;
    end

    methods(Access=private)
        function newObj = HoopsExchangeWrapper()
        %HoopsExchangeWrapper Initialize HoopsExchange object
            newObj.HoopsExchangeMCOS = robotics.internal.HoopsExchange;
        end
    end

    methods(Static)
        function obj = getInstance()
        %getInstance Static method returns singleton instance of
        %wrapper class
            persistent uniqueInstance
            if isempty(uniqueInstance)
                obj = robotics.utils.internal.HoopsExchangeWrapper();
                uniqueInstance = obj;
            else
                obj = uniqueInstance;
            end
            mlock
        end
    end

    methods % Public Access
        function statusCode = hoopsConvert(obj, infile, outfile)
        %HOOPSCONVERT Convert DAE CAD files to STL format
        %   HOOPSCONVERT converts the DAE file defined by INFILE to STL file
        %   defined by OUTFILE. The STL file is written to the path defined by
        %   OUTFILE.

        %   Copyright 2021 The MathWorks, Inc.

            narginchk(3,3);

            validateattributes(infile, {'string', 'char'}, {'scalartext', 'nonempty'}, 'hoopsConvert', 'infile');
            validateattributes(outfile, {'string', 'char'}, {'scalartext', 'nonempty'}, 'hoopsConvert', 'outfile');

            %Perform conversion
            HOOPSStatusCode = obj.HoopsExchangeMCOS.HOOPSConvert(infile, outfile);

            switch(HOOPSStatusCode)
                %The positive status codes are the ones the match the errors from
                %matlab.internal.meshio.stlread. The negative error codes are the
                %errors specific to HOOPS Exchange

              case int32(0) %A3D_SUCCESS
                statusCode = 0;
              case int32(-10002) %A3D_LOAD_CANNOT_ACCESS_CADFILE
                statusCode = 2; %File cannot be opened
              case int32(-10400) %A3D_WRITE_ERROR
                statusCode = robotics.utils.internal.hoopsErrorCodes.HOOPS_WRITE_ERROR; %Corrupted or Empty CAD file
              case int32(-10402) %A3D_WRITE_WRITER_NOT_IMPLEMENTED
                statusCode = robotics.utils.internal.hoopsErrorCodes.HOOPS_WRITER_NOT_IMPLEMENTED; %Output file format not supported
              case int32(-10005) %A3D_LOAD_INVALID_FILE_FORMAT
                statusCode = robotics.utils.internal.hoopsErrorCodes.HOOPS_LOAD_INVALID_FILE_FORMAT; %Invalid input file
              otherwise %Any other errors
                statusCode = -1;
            end
        end
    end

end
