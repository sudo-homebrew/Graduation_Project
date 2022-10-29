function capGroup = showCapsule3D(varargin)
% This function is for internal use only. It may be removed in the future.

%showCapsule3D Displays a 3D capsule transformed to one or more states
%
%   CAPGROUP = showCapsule3D(AX, NUMCIRCLEPTS, CAPGEOMETRY, CAPSTATES) Displays a
%   3D Capsule at one or more locations. CAPGEOMETRY is a struct with the 
%   fields 'Length','Radius', and 'FixedTransform', and CAPSTATES is an
%   N-by-6 [x y z Oz Oy Ox] or N-by-7 [x y z qW qX qY qZ] matrix of states.
%   For each state provided, this function will create an 
%   hgtransform->hgtransform->patch hierarchy, and parent it to CAPGROUP.
%
%   CAPGROUP = showCapsule3D(AX, NUMCIRCLEPTS, CAPGEOMETRY, CAPSTATES, CAPGROUP)
%   If the optional CAPGROUP argument is provided, this function will
%   update the group hierarchy, or create new patch objects underneath the
%   existing group. CAPGROUP is of the form hggroup->hgtransform->hgtransform->patch.

%   Copyright 2020 The MathWorks, Inc.

    ax = varargin{1};
    numCirclePts = varargin{2};
    capGeometry = varargin{3};
    capStates = varargin{4};
    n = 12;
    
    reusePatches = false;
    
    if nargin == 4
        %showCapsule3D(ax, capGeometry, capStates)
        capGroup = hggroup('Parent',ax);
    else
        %showCapsule3D(ax, capGeometry, capStates, capGroup)
        capGroup = varargin{5};
        V = capGroup.Children(1).Children.Children.Vertices;
        numFaces = 4*(n*numCirclePts+n-1);
        facesMatch = (numFaces == size(capGroup.Children(1).Children.Children.Faces,1));
        sameRadius = (V(1,1) == capGeometry.Length);
        sameLength = (V(1,2) == capGeometry.Radius);
        
        if facesMatch && sameRadius && sameLength
            % Can directly reuse existing capsules
            reusePatches = true;
        end
    end
    
    if reusePatches
        % Can directly reuse existing capsules
        capHG = capGroup.Children(1);
    else
        % Calculate Face/Vertices for given Capsule geometry
        [F,V] = createPatch(capGeometry.Length, capGeometry.Radius, numCirclePts, n);
        
        % Create display hierarchy
        capHG = hgtransform(ax);
        capHG.Parent = capGroup;
        capTransformLocal = hgtransform(ax);
        capTransformLocal.Parent = capHG;
        capTransformLocal.Matrix = capGeometry.FixedTransform;
        patch('Faces',F,'Vertices',V,'Parent',capTransformLocal,'FaceColor','b','EdgeColor','none');
        
        % Set up for display in legend
        baseCapWorld = capGroup.Children(1);
        baseCapWorld.Annotation.LegendInformation.IconDisplayStyle = 'children';
        baseCapLocal = baseCapWorld.Children;
        baseCapLocal.Annotation.LegendInformation.IconDisplayStyle = 'children';
        baseCapPatch = baseCapLocal.Children;
        baseCapPatch.Annotation.LegendInformation.IconDisplayStyle = 'on';
    end
    
    % Find number of currently plotted capsules belonging to group
    numPatches = numel(capGroup.Children);
    numStates = size(capStates,1);
    
    % Create more hgpatches if required, or removed extra
    if numPatches == numStates
        % Do nothing
    elseif numPatches < numStates
        % Create new hgtransform objects
        for i = 1:(numStates - numPatches)
            newCap = copy(capHG);
            set(newCap,'Parent',capGroup);
        end
    else
        delete(capGroup.Children(numStates+1:end));
    end
    
    % Get the world transformation matrices for the current set of capsules
    if size(capStates,2) == 6
        R = eul2rotm(capStates(:,4:6));
    else
        R = quat2rotm(capStates(:,4:7));
    end
    
    
    for i = 1:numStates
        capGroup.Children(i).Matrix = [R(:,:,i) capStates(i,1:3)'; 0 0 0 1];
    end
end

function [F,V] = createPatch(L,R,p,is2D)
    % Represent hemisphere using slats
    %
    n = 12;
    tht = linspace(0,pi,1+p).';
    %
    if nargin == 3 && is2D == true
        n = 1/2;
        ang1 = pi/2;
    else
        ang1 = linspace(0,pi,2*n);
    end
    ang2 = ang1;
    x = sin(tht)*sin(ang2)*R;
    y = cos(tht)*ones(1,2*n)*R;
    z = sin(tht)*cos(ang1)*R;
    
    VSphere = [x(:),y(:),z(:)];
    
    % Number of vertices in sphere
    nS = size(VSphere,1);
    
    FSphere = (1:(nS-p-2))';
    FSphere(:,2:4) = [1+FSphere,FSphere+2+p,FSphere+1+p];
    
    % Combine top/bottom vertices to form ends of capsule
    if L >= 0
        V = [x(:)+L, y(:), z(:); -x(:), y(:), z(:)];
    else
        V = [-x(:)+L, y(:), z(:); x(:), y(:), z(:)];
    end
    
    % Represent cylinder with correct number of faces
    FCylinder = (1:p)';
    FCylinder(:,2:4) = [FCylinder+1 FCylinder+nS+1 FCylinder+nS];
    FCylinder = [FCylinder;FCylinder+nS-p-1];
    
    % Represent final patch using faces
    F = [FSphere;FSphere+size(VSphere,1);FCylinder];
end
