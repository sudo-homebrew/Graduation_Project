classdef PointCloud2 < ros.msggen.sensor_msgs.PointCloud2 ...
        & ros.msg.sensor_msgs.internal.PointCloudInterface
    %PointCloud2 Custom MATLAB implementation of sensor_msgs/PointCloud2 type
    %   This class handles ROS point clouds and provides conversion
    %   functions to MATLAB-compatible data structures.
    %
    %   PointCloud2 properties:
    %      PreserveStructureOnRead - Determines if read methods preserve structure
    %
    %   PointCloud2 methods:
    %      readXYZ            - Return 3D point coordinates (x-y-z)
    %      readRGB            - Return per-point color information
    %      readAllFieldNames  - Return all available field names
    %      readField          - Read an arbitrary point field
    %      scatter3           - Method for displaying a 3D point cloud
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties
        %PreserveStructureOnRead - Determines if read methods preserve structure
        %   The point cloud data may be organized 2D (image-like) or 1D
        %   (unordered). Point clouds organized as 2D images may be produced by
        %   depth sensors or stereo cameras.
        %   If this property is set to true, then the output of the read methods
        %   will preserve this organizational structure of the point cloud.
        %   The output will be matrices with size MxNxD, where M is the height
        %   of the organized point cloud, N is its width, and D is the number of
        %   return values per point.
        %   If the property is false, all values will be returned in NxD
        %   matrices. In this case, organized 2D point clouds will be
        %   flattened.
        %   This property is only relevant if the point cloud is
        %   organized (both width and height have values that are greater
        %   than 1).
        %
        %   Default: false.
        PreserveStructureOnRead = false
    end
    
    properties (Access = private)
        %TypeConversion - Object handling ROS <--> MATLAB type conversions
        TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types
        
        %Reader
        Reader = ros.msg.sensor_msgs.internal.PointCloud2Reader
    end
    
    methods
        function obj = PointCloud2(varargin)
            %PointCloud2 Constructor
            %   The arguments feed straight into the generated PointCloud2 class.
           
            obj@ros.msggen.sensor_msgs.PointCloud2(varargin{:});
        end
        
        function scatterHandle = scatter3(obj, varargin)
            %SCATTER3 Custom point cloud visualization
            %   SCATTER3(OBJ) will display the point cloud as a 3D scatter plot
            %   in the current axes handle (or create a new one). If the data 
            %   contains RGB information for each point, the scatter plot will 
            %   be colored accordingly.
            %
            %   SCATTER3(___, Name, Value) allows the specification of
            %   optional name/value pairs to control the plotting.
            %   Potential name/value pairs are:
            %      'Parent'    -  specifies the parent axes object in which the
            %                     point cloud should be drawn.
            %      'MarkerEdgeColor' -  override the default color of the
            %                     points in the scatter plot. This will
            %                     apply a uniform color to all points. The
            %                     color value can be one of the supported
            %                     strings or an RGB vector.
            %
            %   H = SCATTER3(___) returns the scatter series object. Use H
            %   to modify properties of the scatter series after it is created.
            %
            %   The axis orientation of the scatter plot will follow the
            %   standard ROS convention. By default, the x axis is forward, 
            %   the y axis is left, and the z axis is up. If the point
            %   cloud was captured by a device that uses the "_optical"
            %   string in its FrameId, the axis convention changes to z forward, 
            %   x right, and y down.                
            %
            %   Example:
            %      % Show 3D points (might use RGB color if present)
            %      scatter3(ptcloud);
            %
            %      % Show 3D points with uniform red color
            %      scatter3(ptcloud,'MarkerEdgeColor',[1 0 0]);
            
            
            % Parse input arguments
            defaults.Parent = [];
            defaults.MarkerEdgeColor = 'invalid';
            args = obj.parseScatterArguments(defaults, varargin{:});
            
            % Temporarily disable the preservation of structure
            structureSave = obj.PreserveStructureOnRead;
            c = onCleanup(@() cleanup(obj, structureSave));
            obj.PreserveStructureOnRead = false;
            
            % Get XYZ coordinates
            xyz = obj.readXYZ;
            
            % Determine color of point cloud and call scatter3
            sHandle = colorScatter(obj, xyz, defaults, args);
            
            % Properly scale the data
            axis(args.Parent, 'equal');
            grid(args.Parent, 'on');
            title(args.Parent, message('ros:mlroscpp:pointcloud:PointCloud').getString);
            xlabel(args.Parent, 'X');
            ylabel(args.Parent, 'Y');
            zlabel(args.Parent, 'Z');
            
            % Show the camera toolbar and enable the orbit camera
            figureHandle = ancestor(args.Parent, 'figure');
            cameratoolbar(figureHandle, 'Show');
            cameratoolbar(figureHandle, 'SetMode', 'orbit');
            cameratoolbar(figureHandle, 'ResetCamera');
            
            % See http://www.ros.org/reps/rep-0103.html#axis-orientation
            % for information on ROS coordinate systems
            if ~isempty(strfind(obj.Header.FrameId, '_optical'))
                % For point clouds recorded from a camera system, a
                % different axis orientation applies: z forward, x right,
                % and y down.                
                
                % Change the principal axis to -Y and show a good
                % perspective view
                cameratoolbar(figureHandle, 'SetCoordSys','Y');
                set(args.Parent, 'CameraUpVector', [0 -1 0]);
                camorbit(args.Parent, -110, -15, 'data', [0 1 0]);
            else
                % By default, use the standard ROS convention: x forward, y
                % left, and z up.
                cameratoolbar(figureHandle, 'SetCoordSys','Z');
                set(args.Parent, 'CameraUpVector', [0 0 1]);
                camorbit(args.Parent, -70, -15, 'data', [0 0 1]);
            end
            
            % Use the rotation icon instead of the standard pointer
            ptrData = setptr('rotate');
            set(figureHandle, ptrData{:});            
            
            % Return the scatter handle if requested by the user
            if nargout > 0
                scatterHandle = sHandle;
            end
            
            function cleanup(obj, savedPropValue)
                %cleanup Restore structure preservation property
                obj.PreserveStructureOnRead = savedPropValue;
            end
            
            function rgbExists = hasRGBData(obj)
                %hasRGBData Checks if this point cloud contains RGB data
                try
                    obj.getFieldNameIndex('rgb');
                    rgbExists = true;
                catch
                    rgbExists = false;
                end
            end
            
            function scatterHandle = colorScatter(obj, xyz, defaults, args)
                %colorScatter Determine point cloud color and call scatter3
                %   This will be based on user input and the existence of
                %   RGB data in the point cloud.
                
                % If user specified a marker edge color use it for the scatter
                % plot
                if ~strcmp(args.MarkerEdgeColor, defaults.MarkerEdgeColor)
                    color = args.MarkerEdgeColor;
                    scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                        xyz(:,3), 1, '.', 'MarkerEdgeColor', color);
                    return;
                end
                
                % Use point RGB color information (if it exists)
                if hasRGBData(obj)
                    color = obj.readRGB;
                    scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                        xyz(:,3), 1, color, '.');
                    return;
                end
                
                % If no color specified by user, or no RGB information
                % available, use scatter3 defaults
                scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                    xyz(:,3), 1, '.');
            end
            
        end
        
        function xyz = readXYZ(obj)
            %readXYZ Returns the (x,y,z) coordinates of all points
            %   XYZ = readXYZ(OBJ) extracts the (x,y,z) coordinates from
            %   all points in the point cloud object OBJ and returns them
            %   as an Nx3 or HxWx3 matrix of 3D point
            %   coordinates. Each 3-vector represents one 3D point.
            %
            %   If the point cloud contains N points, the returned matrix
            %   has Nx3 elements (or a HxWx3 matrix if the PreserveStructure
            %   property is set to true).
            %
            %   If the point cloud does not contain the 'x', 'y' and 'z'
            %   fields, this function will display an error.
            %
            %
            %   Example:
            %
            %      % Retrieve 3D point coordinates
            %      xyz = readXYZ(ptcloud);
            %
            %      % Plot the points with a 3D scatter plot
            %      scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.');
            
            try
                % Get field indices for X, Y, and Z coordinates
                xIndex = obj.getFieldNameIndex('x');
                yIndex = obj.getFieldNameIndex('y');
                zIndex = obj.getFieldNameIndex('z');
            catch ex
                newex = MException(message('ros:mlroscpp:pointcloud:InvalidXYZData'));
                throw(newex.addCause(ex));
            end
            
            xyz = obj.Reader.readXYZ(obj, xIndex, yIndex, zIndex);
            
            % Reshape the output if requested by the user
            if obj.reshapeOutput
                xyz = reshape(xyz, obj.Width, obj.Height, 3);
                xyz = permute(xyz, [2 1 3]);
            end

        end
        
        function rgb = readRGB(obj)
            %readRGB Returns the RGB color matrix for all points
            %   RGB = readRGB(OBJ) extracts the (r,g,b) for all points in
            %   the point cloud object OBJ and returns them as
            %   an Nx3 or HxWx3 matrix of RGB color values. 
            %   Each 3-vector represents one RGB reading.
            %
            %   If the point cloud contains N points, and color information
            %   for the points is stored in the message, the returned matrix
            %   has Nx3 elements. An HxWx3 matrix is returned if the PreserveStructure
            %   property is set to true. 
            %
            %   This function will display an error if no RGB data is 
            %   stored in the point cloud.
            %
            %   Each RGB value is represented as a double in the range of
            %   [0,1].
            %
            %
            %   Example:
            %      % Retrieve 3D point coordinates
            %      xyz = readXYZ(ptcloud);
            %
            %      % Read corresponding color values
            %      rgb = readRGB(ptcloud);
            %
            %      % Plot the colored points in a 3D scatter plot
            %      scatter3(xyz(:,1), xyz(:,2), xyz(:,3), 1, rgb, '.');
            
            % Get field index for the RGB field
            try
                rgbIdx = obj.getFieldNameIndex('rgb');
            catch ex
                newex = MException(message('ros:mlroscpp:pointcloud:NoColorData'));
                throw(newex.addCause(ex));
            end
            rgb = obj.Reader.readRGB(obj, rgbIdx);
            
            % Reshape the output if requested by the user
            if obj.reshapeOutput
                % Since image is row-major, this is a two-step process
                rgb = reshape(rgb, obj.Width, obj.Height, 3);
                rgb = permute(rgb, [2 1 3]);
            end

        end
        
        function fieldNames = readAllFieldNames(obj)
            %readAllFieldNames - Return all available field names
            %   FIELDNAMES = readAllFieldNames(OBJ) returns the names of
            %   all point fields that are stored in message OBJ. FIELDNAMES
            %   is a 1xN cell array of strings, where N is the number of fields.
            %   If no fields are stored in the message, the return will be
            %   an empty cell array.
            
            % Some of the allowable field names are documented on the ROS
            % Wiki: http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names
            
            fieldNames = {};
            
            % If no point fields are available, return
            if isempty(obj.Fields)
                return;
            end
            
            fieldNames = {obj.Fields.Name};
        end
        
        function fieldData = readField(obj, fieldName)
            %readField Read data based on given field name
            %   FIELDDATA = readField(OBJ,'FIELDNAME') reads the data from
            %   the point field with name 'FIELDNAME' and returns it in the
            %   FIELDDATA variable. If 'FIELDNAME' does not exist, the
            %   function will display an error.
            %
            %   This function returns an NxC vector of values (or a HxWxC
            %   matrix if the PreserveStructure property is set to true)
            %   N is the number of points in the point cloud and C is the number of
            %   values that is assigned for every point in this field. In
            %   most cases, C will be 1.
            %
            %
            %   Example:
            %      % Retrieve the X coordinates of all points
            %      x = readField(ptcloud,'x');
            %
            %   See also readAllFieldNames.
            
            fieldName = convertStringsToChars(fieldName);
            validateattributes(fieldName, {'char', 'string'}, {'nonempty'}, 'readField', ...
                'fieldName');
            
            % Get index of field. This will display an error if the field
            % name does not exist.
            fieldIdx = obj.getFieldNameIndex(fieldName);
            
            fieldData = obj.Reader.readField(obj, fieldIdx);
            
            % Reshape the output if requested by the user
            if obj.reshapeOutput
                count = obj.getFieldCount(fieldIdx);
                if count > 1
                    fieldData = reshape(fieldData, obj.Width, obj.Height, count);
                    fieldData = permute(fieldData, [2 1 3]);
                else
                    fieldData = reshape(fieldData, obj.Width, obj.Height).';
                end
            end

        end
        
        function set.PreserveStructureOnRead(obj, value)
            %set.PreserveStructureOnRead Custom setter for property
            
            validateattributes(value, {'numeric', 'logical'}, ...
                {'scalar','nonempty','binary'}, ...
                'PointCloud2', 'PreserveStructureOnRead');
            
            obj.PreserveStructureOnRead = logical(value);
        end
    end
    
    methods (Access = {?ros.internal.TransformHelper, ?matlab.unittest.TestCase})
        function writeXYZ(obj, xyz)
            %writeXYZ Write points in (x,y,z) coordinates to the message
            %   writeXYZ(OBJ, XYZ) writes the Nx3 matrix of 3D point
            %   coordinates to the point cloud message object OBJ.
            %   Each input row in XYZ is a 3-vector representing one 3D
            %   point. The number of rows of XYZ needs to be equal to the
            %   number of points currently stored in OBJ.
            %
            %   This function completely overwrite any existing XYZ point
            %   information and replaces it with the points given in XYZ. 
            
            validateattributes(xyz, {'numeric'}, {'nonempty', 'ncols', 3, 'nrows', ...
                obj.Height * obj.Width}, 'writeXYZ', 'xyz');
            
            % Get PointField information for x, y, and z data. If these do
            % not exist an error will be displayed. In its current usage,
            % this method is always called after readXYZ, so the fields
            % should exist.
            xFieldIdx = obj.getFieldNameIndex('x');
            yFieldIdx = obj.getFieldNameIndex('y');
            zFieldIdx = obj.getFieldNameIndex('z');
            
            % Create copy of byte array for storing the new point information
            % For maximum performance, we are writing to a local copy and
            % not to obj.Data directly.
            data = obj.Data;
            
            % Get byte index only once (this is expensive)
            byteIdx = obj.Reader.getByteIndexForField(obj, xFieldIdx).';
            
            % Calculate the byte offsets for the different fields
            % This helps with performance, since we can re-use the
            % already calculated byte index
            xOff = double(obj.Fields(xFieldIdx).Offset);
            yOff = obj.Reader.relativeFieldOffset(obj, xOff, yFieldIdx);
            zOff = obj.Reader.relativeFieldOffset(obj, xOff, zFieldIdx);
            
            % Write XYZ data. Do this in a nested function, so the full
            % data array does not have to be copied around
            writeDataToField(obj, xFieldIdx, xyz(:,1), byteIdx);
            writeDataToField(obj, yFieldIdx, xyz(:,2), byteIdx + yOff);
            writeDataToField(obj, zFieldIdx, xyz(:,3), byteIdx + zOff);
            
            % Write byte array back to message Data property
            obj.Data = data;
            
            function writeDataToField(obj, fieldIdx, fieldData, byteIdx)
                %writeDataToField Write data to a point field
                
                % Recover MATLAB data type of field elements
                mlFieldType = obj.TypeConversion.rosToMATLABType(obj.Fields(fieldIdx).Datatype);
                
                % Cast the input to the expected type
                if ~isa(fieldData, mlFieldType)
                    fieldData = cast(fieldData, mlFieldType);
                end
                
                % Treat field data as byte array
                fieldDataBytes = typecast(fieldData, 'uint8');

                % Now write the field data to the point field
                data(byteIdx) = fieldDataBytes;
            end
        end
        
    end
    
    methods (Static, Access = private)
        function args = parseScatterArguments(defaults, varargin)
            %parseScatterArguments Parse arguments for scatter3 function
            
            % Parse inputs
            parser = inputParser;
            
            % Cannot make any assumptions about data type
            addParameter(parser, 'Parent', defaults.Parent);
            
            addParameter(parser, 'MarkerEdgeColor', defaults.MarkerEdgeColor, ...
                @(x) validateattributes(x, {'numeric', 'char'}, ...
                {}, 'PointCloud2', 'MarkerEdgeColor'));
            
            % Return data
            parse(parser, varargin{:});
            args = parser.Results;
            
            % Delay the preparation of a new plot (and potentially the creation 
            % of a new window until all other inputs have been parsed)
            if isnumeric(args.Parent) && isequal(args.Parent, defaults.Parent)
                args.Parent = newplot;
            end
            
            % Check that parent is a graphics Axes handle
            robotics.internal.validation.validateAxesHandle(args.Parent, ...
                'ros:mlroscpp:pointcloud:ParentNotHandle');            
        end
    end
    
    methods (Access = protected)        
        function applyReshape = reshapeOutput(obj)
            %reshapeOutput Check if output should be reshaped
            %   If the point cloud is ordered and the user specified that
            %   the structure should be preserved on reading, then the
            %   output of read functions needs to be reshaped.
            
            if obj.PreserveStructureOnRead && obj.Height ~= 1
                applyReshape = true;
            else
                applyReshape = false;
            end
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method of base class
            cpObj = copyElement@ros.msggen.sensor_msgs.PointCloud2(obj);
            
            % Copy additional property
            cpObj.PreserveStructureOnRead = obj.PreserveStructureOnRead;            
        end
    end

    methods (Access = ?ros.Message)
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            
            reload@ros.msggen.sensor_msgs.PointCloud2(obj, strObj);
            if isfield(strObj, 'PreserveStructureOnRead')
                obj.PreserveStructureOnRead = strObj.PreserveStructureOnRead;
            else
                obj.PreserveStructureOnRead = false;
            end
        end
    end
    
    methods (Access = {...
            ?ros.msg.sensor_msgs.internal.PointCloudInterface, ...
            ?ros.msg.sensor_msgs.internal.PointCloud2Reader, ...
            ?matlab.unittest.TestCase})
        function fieldIdx = getFieldNameIndex(obj, fieldName)
            %getFieldNameIndex Get index of field in PointField array
            
            allFieldNames = obj.readAllFieldNames;
            [isValid, fieldIdx] = ismember(fieldName, allFieldNames);
            if ~isValid
                error(message('ros:mlroscpp:pointcloud:InvalidFieldName', ...
                    fieldName, strjoin(allFieldNames, ', ')));
            end
        end
        
        function offset = getFieldOffset(obj, fieldIdx)
            offset = obj.Fields(fieldIdx).Offset;
        end
        
        function datatype = getFieldDatatype(obj, fieldIdx)
            datatype = obj.Fields(fieldIdx).Datatype;
        end
        
        function count = getFieldCount(obj, fieldIdx)
            count = obj.Fields(fieldIdx).Count;
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj = saveobj@ros.msggen.sensor_msgs.PointCloud2(obj);
            strObj.PreserveStructureOnRead = obj.PreserveStructureOnRead;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.sensor_msgs.PointCloud2.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.msg.sensor_msgs.PointCloud2(strObj);
        end
    end
end
