function pos = retrieveAdjustedPlotBoxPosition(axHandle)
%This function is for internal use only. It may be removed in the future.

%retrieveAdjustedPlotBoxPosition Returns the position of the plotted axis region
%
% pos = retrieveAdjustedPlotBoxPosition(axHandle)
%
% This function returns the position of the plotted region of an axis,
% which may differ from the actual axis position, depending on the axis
% limits, data aspect ratio, and plot box aspect ratio.  The position is
% returned in the same units as the those used to define the axis itself.
% This function can only be used for a 2D plot.
%
% Input variables:
%
%   axHandle - Axis handle of a 2D axis (if ommitted, current axis is used).
%
% Output variables:
%
%   pos - Four-element position vector, in same units as axHandle
%
% Example:
%       load exampleMaps.mat;
%       map = binaryOccupancyMap(simpleMap, 2);
%       show(map);
% 
%       % Get current axes handle
%       ax = gca;
% 
%       % Get axis position using ax.Position and plot the axis region
%       h1 = annotation('rectangle', ax.Position, 'edgecolor', 'b', 'linestyle', '--', 'lineWidth', 5);
% 
%       % Get axis position using retrieveAdjustedPlotBoxPosition and plot the axis region
%       h2 = annotation('rectangle', nav.algs.internal.retrieveAdjustedPlotBoxPosition(ax),...
%       'edgecolor', 'r', 'linestyle', '--', 'lineWidth', 5);

% Copyright 2019 The MathWorks, Inc.

if nargin < 1
    axHandle = gca;
end

if ~ishandle(axHandle) || ~strcmp(get(axHandle,'type'), 'axes')
    error('Input must be an axis handle');
end

% Get position of axis in pixels
currunit = get(axHandle, 'units');
set(axHandle, 'units', 'pixels');
axisPos = get(axHandle, 'Position');
set(axHandle, 'Units', currunit);

% Calculate box position based axis limits and aspect ratios
darismanual  = strcmpi(get(axHandle, 'DataAspectRatioMode'),    'manual');
pbarismanual = strcmpi(get(axHandle, 'PlotBoxAspectRatioMode'), 'manual');

if ~darismanual && ~pbarismanual 
    pos = axisPos;   
else
    xlim = get(axHandle, 'XLim');
    ylim = get(axHandle, 'YLim');
    
    % Deal with axis limits auto-set via Inf/-Inf use    
    if any(isinf([xlim ylim]))
        hc = get(axHandle, 'Children');
        hc(~arrayfun( @(axHandle) isprop(axHandle, 'XData' ) & isprop(axHandle, 'YData' ), hc)) = [];
        xdata = get(hc, 'XData');
        if iscell(xdata)
            xdata = cellfun(@(x) x(:), xdata, 'uni', 0);
            xdata = cat(1, xdata{:});
        end
        ydata = get(hc, 'YData');
        if iscell(ydata)
            ydata = cellfun(@(x) x(:), ydata, 'uni', 0);
            ydata = cat(1, ydata{:});
        end
        isplotted = ~isinf(xdata) & ~isnan(xdata) & ...
                    ~isinf(ydata) & ~isnan(ydata);
        xdata = xdata(isplotted);
        ydata = ydata(isplotted);
        if isempty(xdata)
            xdata = [0 1];
        end
        if isempty(ydata)
            ydata = [0 1];
        end
        if isinf(xlim(1))
            xlim(1) = min(xdata);
        end
        if isinf(xlim(2))
            xlim(2) = max(xdata);
        end
        if isinf(ylim(1))
            ylim(1) = min(ydata);
        end
        if isinf(ylim(2))
            ylim(2) = max(ydata);
        end
    end

    dx = diff(xlim);
    dy = diff(ylim);
    dar = get(axHandle, 'DataAspectRatio');
    pbar = get(axHandle, 'PlotBoxAspectRatio');

    limDarRatio = (dx/dar(1))/(dy/dar(2));
    pbarRatio = pbar(1)/pbar(2);
    axisRatio = axisPos(3)/axisPos(4);

    if darismanual
        if limDarRatio > axisRatio
            pos(1) = axisPos(1);
            pos(3) = axisPos(3);
            pos(4) = axisPos(3)/limDarRatio;
            pos(2) = (axisPos(4) - pos(4))/2 + axisPos(2);
        else
            pos(2) = axisPos(2);
            pos(4) = axisPos(4);
            pos(3) = axisPos(4) * limDarRatio;
            pos(1) = (axisPos(3) - pos(3))/2 + axisPos(1);
        end
    elseif pbarismanual
        if pbarRatio > axisRatio
            pos(1) = axisPos(1);
            pos(3) = axisPos(3);
            pos(4) = axisPos(3)/pbarRatio;
            pos(2) = (axisPos(4) - pos(4))/2 + axisPos(2);
        else
            pos(2) = axisPos(2);
            pos(4) = axisPos(4);
            pos(3) = axisPos(4) * pbarRatio;
            pos(1) = (axisPos(3) - pos(3))/2 + axisPos(1);
        end
    end
end

% Convert plot box position to the units used by the axis

hparent = get(axHandle, 'parent');
hfig = ancestor(hparent, 'figure'); % in case in panel or similar
currax = get(hfig, 'currentaxes');

temp = axes('Units', 'Pixels', 'Position', pos, 'Visible', 'off', 'parent', hparent);
set(temp, 'Units', currunit);
pos = get(temp, 'position');
delete(temp);

set(hfig, 'currentaxes', currax);
