function capGroup = showCapsule2D(axHandle, numCirclePts, P, V, L, R, capGroup)
% This function is for internal use only. It may be removed in the future.

%showCapsule2D Displays capsules defined by their start point,
%unit-orientation vector, center-line length, and radius
%
%   CAPGROUP = showCapsule2D(AXHANDLE, NUMCIRCLEPTS, P, V, L, R) Plots a
%   capsule at multiple poses. A Capsule is defined by L, length,
%   and radius, R, both scalars, and a set of 2-by-N matrices, P and V,
%   where P represents the beginning of the line segment at the capsule
%   center, and V is a unit vector pointing along the line segment. Separate
%   states are plotted using a single patch object, where each sub-patch
%   consists of NUMCIRCLEPTS*2 vertices, and a single face.
%
%   CAPGROUP = showCapsule2D(AXHANDLE, NUMCIRCLEPTS, P, V, L, R, CAPGROUP)
%   If the optional CAPGROUP argument is provided, this function will
%   update the patch at the bottom of the hg hierarchy rather than creating
%   a new patch. CAPGROUP is of the form hggroup->hgtransform->hgtransform->patch.

%   Copyright 2020-2021 The MathWorks, Inc.

    numCap = size(V,2);

    xCenter = P(1,:)+[0; 1]*V(1,:).*L;
    yCenter = P(2,:)+[0; 1]*V(2,:).*L;

    circAngles = linspace(pi/2,-pi/2,numCirclePts)';
    thetas = circAngles+atan2(V(2,:),V(1,:));
    cT = cos(thetas).*R;
    sT = sin(thetas).*R;
    xCircles = reshape([-cT; cT]*sign(L),numCirclePts,[])+xCenter(:)';
    yCircles = reshape([-sT; sT]*sign(L),numCirclePts,[])+yCenter(:)';
    
    F = (0:size(V,2)-1)'*(numCirclePts*2)+(1:numCirclePts*2);
    
    if nargin == 7
        % Grab handle to patch
        p = capGroup.Children.Children.Children;
        set(p,'XData',xCircles,'YData',yCircles,'Faces',F,'CData',nan(numCap,1,3));
    else
        capGroup = hggroup(axHandle);
        capT1 = hgtransform(axHandle);
        capT1.Parent = capGroup;
        capT1.Annotation.LegendInformation.IconDisplayStyle = 'children';
        capT2 = hgtransform(axHandle);
        capT2.Parent = capT1;
        capT2.Annotation.LegendInformation.IconDisplayStyle = 'children';
        p = patch('Parent',capT2,'XData',xCircles,'YData',yCircles,'Faces',F,'FaceColor','flat','EdgeColor',[.4 .4 .4],'CData',nan(numCap,1,3));
        p.Annotation.LegendInformation.IconDisplayStyle = 'on';
    end
end
