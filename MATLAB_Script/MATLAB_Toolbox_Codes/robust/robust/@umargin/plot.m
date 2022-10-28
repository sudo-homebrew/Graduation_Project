function plot(blk)
%PLOT  Visualize gain and phase uncertainty for UMARGIN block.
%
%   For a UMARGIN block BLK, PLOT(BLK) plots
%
%     * The gain and phase uncertainty range. This shows the maximum
%       gain-only and phase-only variations (gain and phase margins)
%       and the combined gain and phase variations in between.
%
%     * The disk of values taken by the dynamic, multiplicative factor 
%       F(s) modeling gain and phase uncertainty. For SISO loops, this 
%       factor changes the open-loop response L to L*F.
%
%   See also UMARGIN, DISKMARGINPLOT.

%   Copyright 1986-2020 The MathWorks, Inc.
f = gcf;
clf(f)
DGM = blk.GainChange;
ax = subplot(121);
diskmarginplot(DGM)
xlabel(ax,getString(message('Robust:umodel:umargin9')))
ylabel(ax,getString(message('Robust:umodel:umargin10')))
subplot(122)
diskmarginplot(DGM,'disk')