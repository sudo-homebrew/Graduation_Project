classdef MapIO
%This class is for internal use only. It may be removed in the future.

%   The class has static methods to read/write binary (BT) and octree (OT)
%   format to/from occupancyMap3D object

%   Copyright 2017-2018 The MathWorks, Inc.

    methods (Static)
        function omap = read(filename)
        %read Import a Binary Tree (BT) or Octree (OT) format 3D occupancy map
        %   OMAP = read(FILENAME) imports a Binary Tree or Octree format file
        %   FILENAME (should contain its extension as well). This function returns
        %   a occupancyMap3D object containing the occupancy values from
        %   the imported file.

            omap = occupancyMap3D;
            % filename verification happens in occupancyMap3D/read method
            omap.read(filename);
        end

        function write(omap, filename)
        %write Write occupancyMap3D contents to a Binary Tree (BT) or Octree (OT) format
        %   write(OMAP, FILENAME) write to a binary (BT) or octree (OT) file with
        %   FILENAME (containing the extension BT/OT) the contents of the specified
        %   OMAP, i.e., occupancyMap3D object.

            validateattributes(omap, {'occupancyMap3D'}, {'scalar', 'nonempty'}, 'write', 'OMAP');
            omap.write(filename);
        end

        function omap = deserializeROSMsgData(isBinary, res, data)
        %deserializeROSMsgData - Return a occupancyMap3D object using
        %ROS message data
        %   OMAP = deserializeROSMsgData(ISBINARY, RES, DATA) returns a
        %   OMAP occupancyMap3D object which is constructed using the
        %   octomap_msgs/Octomap resolution (RES) and DATA. Based on
        %   ISBINARY boolean value the OMAP contains either binary or
        %   full occupancy values.

            omap = occupancyMap3D;
            if isBinary
                omap.Octree.deserializationBinaryROSMsgData(res, data);
            else
                omap.Octree.deserializationFullROSMsgData(res, data);
            end
        end
    end
end
