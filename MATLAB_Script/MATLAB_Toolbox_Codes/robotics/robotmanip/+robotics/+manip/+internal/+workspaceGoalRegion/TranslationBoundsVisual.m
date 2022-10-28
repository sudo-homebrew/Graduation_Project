classdef TranslationBoundsVisual
%This class is for internal use only. It may be removed in the future.

%TranslationBoundsVisual Visualizes a bounding box corresponding to translation bounds

%   Copyright 2020 The MathWorks, Inc.
%#codegen


    properties
        %TrX
        TrX = [0, 0]

        %TrY
        TrY = [0, 0]

        %TrZ
        TrZ = [0, 0]

        %BoundaryWidth
        BoundaryWidth = 1.5

        %BoundarySpec
        BoundarySpec = 'k:'

        %Pose
        Pose = eye(4);

    end

    methods(Access=private)
        function vertices = boundingBoxVertices(obj)
        %boundingBoxVertices Returns the vertices based on the translation bounds
        %   Given the corner min and max points, return the vertices for a
        %   bounding box

            xmin = obj.TrX(1);
            xmax = obj.TrX(2);
            ymin = obj.TrY(1);
            ymax = obj.TrY(2);
            zmin = obj.TrZ(1);
            zmax = obj.TrZ(2);

            % Create the zmin and zmax faces of the bounding box. Next, join
            % these faces. The "nan" vertices act as line separators.
            vertices = [xmin, ymin, zmin; % vertices on the zmin plane
                        xmax, ymin, zmin;
                        xmax, ymax, zmin;
                        xmin, ymax, zmin;
                        xmin, ymin, zmin;
                        nan(1, 3);
                        xmin, ymin, zmax; % vertices on the zmax plane
                        xmax, ymin, zmax;
                        xmax, ymax, zmax;
                        xmin, ymax, zmax;
                        xmin, ymin, zmax;
                        nan(1, 3);
                        xmin, ymin, zmin; % edge between (xmin, ymin) on the zmin and zmax planes
                        xmin, ymin, zmax;
                        nan(1, 3);
                        xmax, ymin, zmin; % edge between (xmax, ymin) on the zmin and zmax planes
                        xmax, ymin, zmax;
                        nan(1, 3);
                        xmax, ymax, zmax; % edge between (xmax, ymax) on the zmin and zmax planes
                        xmax, ymax, zmin;
                        nan(1, 3);
                        xmin, ymax, zmin;  % edge between (xmin, ymax) on the zmin and zmax planes
                        xmin, ymax, zmax;];
        end
    end

    methods
        function t = show(obj, parent)
        %show Visualizes the bounding box and attaches it to an hgtransform
            t = hgtransform('Parent', parent);
            t.Tag = 'DO_NOT_EDIT_TranslationBound';
            vertices = obj.boundingBoxVertices();
            plot3(vertices(:, 1), vertices(:, 2), vertices(:, 3), obj.BoundarySpec, ...
                  'Parent', t, ...
                  'LineWidth', obj.BoundaryWidth);
            set(t, 'Matrix', obj.Pose);
        end
    end
end
