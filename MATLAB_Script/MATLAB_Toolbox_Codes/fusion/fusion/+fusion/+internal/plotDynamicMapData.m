function plotDynamicMapData(L, W, xLocalLimits, yLocalLimits, resolution, gridSize, rgbImage, velData, fastUpdate, plotVelocity, parent)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2021 The MathWorks, Inc.

% Parse parent for plotting
if nargin < 11
    parent = gca;
end

% Make parent ready for plotting
curr = parent.NextPlot;
parent.NextPlot = 'add';

% First try to get the plotters from the parent
hIm = findall(parent,'Type','Image','Tag','dynamicMapOccupancyPlotter');
hVel = findall(parent,'Type','quiver','Tag','dynamicMapVelocityPlotter');
imPlotter = hIm(~isempty(hIm));
velPlotter = hVel(~isempty(hVel));

if (isempty(imPlotter) || ~isvalid(imPlotter))
    imPlotter = imagesc(parent,repmat(zeros(gridSize),[1 1 3]),'Tag','dynamicMapOccupancyPlotter');
    fastUpdate = false;
end

if plotVelocity && (isempty(velPlotter) || ~isvalid(velPlotter))
    velPlotter = quiver(parent,nan,nan,nan,nan,0,'Tag','dynamicMapVelocityPlotter');
    fastUpdate = false;
end

% Set values of impotter and axes
imPlotter.CData = gather(rgbImage);

if ~fastUpdate
    topLeft = [xLocalLimits(1) yLocalLimits(1)] + 1/(2*resolution);
    bottomRight = [xLocalLimits(2) yLocalLimits(2)] - 1/(2*resolution);  
    imPlotter.XData = [topLeft(1) bottomRight(1)];
    imPlotter.YData = [bottomRight(2) topLeft(2)];
    parent.XLim = xLocalLimits;
    parent.YLim = yLocalLimits;
    parent.YDir = 'normal';
end

% Plot velocities;
if plotVelocity
    velPlotter.XData = velData.XData;
    velPlotter.YData = velData.YData;
    velPlotter.UData = velData.UData;
    velPlotter.VData = velData.VData;
end

% Plot the color wheel
plotColorWheel(L, W, xLocalLimits, yLocalLimits, parent);

% Return state of next plot
parent.NextPlot = curr;
end

function plotColorWheel(L, W, xLocalLimits, yLocalLimits, parent)
sf = findall(parent,'Type','surf','Tag','dynamicMapColorWheel');
if ~isempty(sf)
    return;
end

% Location of wheel center
x = xLocalLimits(1) + 0.9*L;
y = yLocalLimits(1) + 0.9*W;

% Radius of the wheel
r = 0.1*min(L,W);

theta = linspace(0,360,360);
range = linspace(0.5*r,r,10);
[T, R] = meshgrid(theta,range);
X = R.*cosd(T) + x;
Y = R.*sind(T) + y;
Z = 1e-3*ones(size(X));

sf = surf(parent,X,Y,Z,'Tag','dynamicMapColorWheel');
h = T./360;
s = R./r;
v = ones(size(R));
rgb = hsv2rgb(cat(3,h,s,v));
sf.CData = rgb;
sf.EdgeColor = 'none';
view(parent,2);
end