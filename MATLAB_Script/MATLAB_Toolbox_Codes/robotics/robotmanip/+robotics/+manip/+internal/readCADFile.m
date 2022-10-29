function result = readCADFile(cadFileName)
%This function is for internal use only. It may be removed in the future.

% readCADFile Import and read CAD files
%   Import and read vertex and face data from CAD files. Non-STL files are
%   converted to STL and then read.
%
%   Copyright 2021 The MathWorks, Inc.

%#codegen

narginchk(1,1)

%Filename and CAD file extension
[~, filename, ext] = fileparts(cadFileName);

%For non-STL files convert to STL
if strcmpi(ext, ".dae")
    %Create a temp directory in the system temp location
    newDir = tempname;
    mkdir(newDir);

    %STL file name in temp directory
    stlFileName = fullfile(newDir, [char(filename), '.stl']);

    %Convert to STL format
    statusCode = robotics.utils.internal.hoopsConvert(cadFileName, stlFileName);

    if statusCode == 0 %SUCCESS
        %Read from the STL file
        result = matlab.internal.meshio.stlread(stlFileName);
        
        %Parse the DAE to fetch the "up_axis" tag
        %If the up_axis is present, HOOPS performs an axis transformation
        %from the x,y,z coordinates defined in the DAE to an intermediate
        %right-up-in coordinate system as defined in the DAE spec in 
        % https://www.khronos.org/files/collada_spec_1_4.pdf.
        %To keep the behaviour of DAE import consistent with commonly used
        %3p simulators, the up_axis tag has to be ignored. To this end, a
        %compensating transformation is being performed here that reverses
        %the x,y,z -> right-up-in transformation.
        
        daeStruct = readstruct(cadFileName, "FileType","xml");
        
        %check if the up_axis tag is defined
        upAxisTagExists = isfield(daeStruct, 'asset') && isfield(daeStruct.asset, 'up_axis');
        
        if upAxisTagExists
            upAxis = daeStruct.asset.up_axis;
            
            V = result.Vertices;
            N = result.Normals;
            
            switch char(upAxis)
                case 'Z_UP'
                    %Rotate vertices and normals 90 degree CCW about x-axis
                    rotX90 = axang2rotm([1 0 0 pi/2]);
                    result.Vertices = V*rotX90';
                    result.Normals = N*rotX90';
                case 'X_UP'
                    %Rotate vertices and normals 90 degree CW about z-axis
                    rotZminus90 = axang2rotm([0 0 1 -pi/2]);
                    result.Vertices = V*rotZminus90';
                    result.Normals = N*rotZminus90';
                otherwise 
                    %For Y_UP, do nothing since the xyz -> rui
                    %transformation doesn't involve any rotation. So, the
                    %rui coordinates defined in the stl file correspond to
                    %the xyz values defined in the DAE.
            end
        end
    else
        %Create a dummy result struct
        result = matlab.internal.meshio.stlread(1);

        %Set the ErrorCode to that returned from hoopsConvert
        result.ErrorCode = statusCode;
    end

    if exist(stlFileName, 'file')
        %Delete the temp STL file
        delete(stlFileName)
    end
else %For STL CAD file
    result = matlab.internal.meshio.stlread(cadFileName);
end

end
