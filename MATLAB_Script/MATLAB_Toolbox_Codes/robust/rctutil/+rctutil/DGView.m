classdef DGView < handle
   % Graphical view of D,G fitting results.
   
   %   Copyright 2019 The MathWorks, Inc.
   
   properties
      Focus    % Frequency focus (for fit)
      FullDG   % See MUSYN option of the same name
      DATA     % Data and fit broken down by block and tab group
      UI       % UI handles
   end
   
   
   methods
      
      function h = DGView(blk,dr,PSI,drw,Drw,Gw,w,widx,blkNames,FULLDG)
         % Constructor
         h.FullDG = FULLDG;
         
         % Map data to tabs (dr, dr\Dr/dr, dc\Gcr/dr)
         h.updateData(blk,blkNames,dr,PSI,drw,Drw,Gw,w,widx);
         
         % Create figure
         f = figure('Units','character','IntegerHandle','off','NumberTitle','off',...
            'Name',getString(message('Robust:design:dgview1')),'Visible','off');
         
         % Populate UI
         h.createUI(f)
         h.populateTab(1,1)
         f.Visible = 'on';
         f.SizeChangedFcn = @(x,y) resizeUI(h);
         f.UserData = h;
      end
      
      function updateData(h,blk,blkNames,dr,PSI,drw,Drw,Gw,w,widx)
         % Maps data to tabs.
         % DATA is partitioned into three tab groups DATA{1} to DATA{3}
         % corresponding to diag(D), off-diagonal D, and jG.
         % Each group contains one struct per block represented in tab group,
         % with a pointer to the tab it appears in. The scalings for repeated
         % blocks always appear in their own tab.
         % Note: Assumes BLK(:,1)>0 if no G
         
         % Focus
         wmin = 10^floor(log10(w(widx(1))));
         wmax = 10^ceil(log10(w(widx(2))));
         h.Focus = [wmin,wmax];
         
         % Fit
         Ts = dr.Ts;
         Mr = size(dr,1);
         Mc = size(PSI,1)-Mr;
         J = blkdiag(eye(Mr),-eye(Mc));
         Q = PSI'*J*PSI;
         D = Q(1:Mr,1:Mr);         % dr\Dr/dr
         jG = Q(Mr+1:Mr+Mc,1:Mr);  % dc\Gcr/dr
         dw = frd(drw,w,Ts);
         Dw = frd(Drw,w,Ts);
         jGw = frd(1i*Gw,w,Ts);
         
         % BLK structure
         nblk = size(blk,1);
         bdim = abs(blk);
         rp = find(blk(:,2)==0);
         bdim(rp,2) = bdim(rp,1);
         blkptr = cumsum([1 1; bdim]);
         Repeat = (blk(:,2)==0 & bdim(:,1)>1);
         hasG = (blk(:,1)<0);
         
         % For each block, build structure keeping track of target tab and
         % plotted data (S1 is for dr, S2 for dr\Dr/dr, S3 for dc\Gcr/dr)
         blkid = num2cell((1:nblk)');
         S1 = struct('Block',blkid,'Name',blkNames,'Tab',[],'Data',[],'Fit',[]);
         S2 = struct('Block',blkid,'Name',blkNames,'Tab',[],'Data',[],'Fit',[]);
         S3 = struct('Block',blkid,'Name',blkNames,'Tab',[],'Data',[],'Fit',[]);
         
         % Note: D,G for repeated blocks are always shown on a tab of their own
         nScalard = sum(~Repeat);
         nScalarG = sum(~Repeat & blk(:,1)<0);
         itabOffset = [ceil(nScalard/6) ceil(nScalarG/4)];
         iScalar = 0; iRepeat = 0;
         for i=1:nblk
            % Block indices and dimensions
            ridx = blkptr(i,2):blkptr(i+1,2)-1;  % index into dr,Dr
            cidx = blkptr(i,1):blkptr(i+1,1)-1;  % index into dc,Dc
            if Repeat(i)
               % Full D,G
               iRepeat = iRepeat+1;
               S1(i).Tab = itabOffset(1)+iRepeat;
               S1(i).Data = dw(ridx);
               S1(i).Fit = sminreal(dr(ridx,ridx));
               if h.FullDG(1)
                  S2(i).Tab = iRepeat;
                  S2(i).Data = Dw(ridx,ridx);
                  S2(i).Fit = sminreal(D(ridx,ridx));
               end
               if hasG(i)
                  S3(i).Tab = itabOffset(2)+iRepeat;
                  S3(i).Data = jGw(cidx,ridx);
                  S3(i).Fit = sminreal(jG(cidx,ridx));
               end
            else
               % Scalar D,G
               ridx = ridx(1);  cidx = cidx(1);
               iScalar = iScalar+1;
               S1(i).Tab = ceil(iScalar/6);
               S1(i).Data = dw(ridx);
               S1(i).Fit = sminreal(dr(ridx,ridx));
               if hasG(i)
                  S3(i).Tab = ceil(iScalar/4);
                  S3(i).Data = jGw(cidx,ridx);
                  S3(i).Fit = jG(cidx,ridx);
               end
            end
         end
         
         h.DATA = {S1 S2(Repeat & h.FullDG(1)) S3(hasG)};
      end
      
      function updateView(h)
         % Updates view when data changed
         iGroup = find(strcmp(get([h.UI.TabGroup],'Visible'),'on'));
         tg = h.UI(iGroup).TabGroup;
         iTab = find(tg.SelectedTab==tg.Children);
         % Clear plots
         for ig=1:3
            Tabs = h.UI(ig).Tabs;
            for it=1:numel(Tabs)
               delete(Tabs(it).Plots)
               h.UI(ig).Tabs(it).Plots = [];
            end
         end
         % Repopulate visible tab
         h.populateTab(iGroup,iTab) %#ok<*FNDSB>
      end
      
      function pf = isValid(h)
         % Return false if figure was deleted
         pf = ishandle(h.UI(1).TabGroup);
      end
      
   end
   
   methods (Access=protected)
      
      function createUI(h,f)
         % Construct UI. This consists of three tab groups with varying
         % number of tabs. Each tab contains up to six Bode plots.
         S = struct('Radio',cell(3,1),'TabGroup',[],'Tabs',[]);
         ntabs = [max([h.DATA{1}.Tab]) max([0 h.DATA{2}.Tab]) max([0 h.DATA{3}.Tab])];
         for ct=1:3
            M = h.DATA{ct};  % dr, D, or G
            rb = uicontrol('Style','radiobutton',...
               'Parent',f,'Units','character',...
               'Callback',@(x,y) showTabGroup(h,ct));
            if ntabs(ct)==0
               rb.Enable = 'off';
            end
            tg = uitabgroup(f,'Units','character','Visible','off',...
               'HandleVisibility','off',...
               'SelectionChangedFcn',@(x,y) localUpdateTab(x,h));
            S(ct).Radio = rb;
            S(ct).TabGroup = tg;
            % Create tabs
            S(ct).Tabs = struct('Tab',cell(ntabs(ct),1),'Plots',[]);
            for k=1:ntabs(ct)
               ix = find([M.Tab]==k);
               if isscalar(ix)
                  label = getString(message('Robust:design:dgview4',M(ix).Block));
               else
                  blkids = sprintf('%d,',[M(ix).Block]);
                  label = getString(message('Robust:design:dgview5',blkids(1:end-1)));
               end
               S(ct).Tabs(k).Tab = uitab(tg,'Units','character','Title',label);
            end
            
         end
         
         % Initialization
         S(1).Radio.String = getString(message('Robust:design:dgview2'));
         S(2).Radio.String = getString(message('Robust:design:dgview3'));
         S(3).Radio.String = 'j G';
         S(1).Radio.Value = 1;
         S(1).TabGroup.Visible = 'on';
         
         % Store handles
         h.UI = S;
         
         % Size properly
         resizeUI(h);
      end
      
      function resizeUI(h)
         % React to figure resize.
         S = h.UI;
         f = S(1).Radio.Parent;
         fw = f.Position(3);  % width
         fh = f.Position(4);  % height
         % Radio buttons
         rw = fw/4;
         x0 = fw/16; y0 = 0.5;
         S(1).Radio.Position = [x0+fw/8 y0 rw 2];
         S(2).Radio.Position = [x0+3*fw/8 y0 rw 2];
         S(3).Radio.Position = [x0+5*fw/8 y0 rw 2];
         % Tab groups
         y0 = y0+2;
         th = fh-y0;
         for ct=1:3
            S(ct).TabGroup.Position = [0 y0 fw th];
         end
      end
      
      function showTabGroup(h,j)
         % Show selected tab group
         S = h.UI;
         % Toggle visibility
         for ct=1:3
            S(ct).Radio.Value = 0;
            S(ct).TabGroup.Visible = 'off';
            set(cat(1,S(ct).Tabs.Plots),'Visible','off')
         end
         % Activate selection
         localUpdateTab(S(j).TabGroup,h)
         S(j).Radio.Value = 1;
         S(j).TabGroup.Visible = 'on';
      end
      
      function populateTab(h,iGroup,iTab)
         % Creates plots
         tS = h.UI(iGroup).Tabs(iTab);
         if ~isempty(tS) && isempty(tS.Plots)
            p = uipanel(tS.Tab);
            S = h.DATA{iGroup};
            S = S([S.Tab]==iTab);
            Repeat = (isscalar(S) && size(S.Fit,1)>1);
            bopt = bodeoptions;
            bopt.Title.FontSize = 10;
            bopt.Title.FontWeight = 'Bold';
            bopt.Grid = 'on';
            bopt.XlimMode = {'manual'};
            bopt.XLim = {h.Focus};
            switch iGroup
               case 1
                  % Diagonal entries of D (max = six/tab)
                  if Repeat
                     n = size(S.Fit,1);
                  else
                     n = numel(S);
                  end
                  nc = round(sqrt(n));
                  nr = ceil(n/nc);
                  bopt.PhaseVisible ='off';
                  for ct=1:n
                     ax = subplot(nr,nc,ct,'Parent',p); %#ok<*AGROW>
                     if Repeat
                        bopt.Title.String = getString(message('Robust:design:dgview6',ct,ct,S.Block,S.Name));
                        d = S.Data(ct);  dfit = S.Fit(ct,ct);
                     else
                        bopt.Title.String = getString(message('Robust:design:dgview7',S(ct).Block,S(ct).Name));
                        d = S(ct).Data;  dfit = S(ct).Fit;
                     end
                     hplot(ct,1) = bodeplot(ax,d'*d,dfit'*dfit,bopt);
                  end
               case 2
                  % Plot full D
                  assert(isscalar(S))
                  bopt.Title.String = getString(message('Robust:design:dgview8',S.Block,S.Name));
                  bopt.PhaseMatching = 'on';
                  bopt.PhaseMatchingFreq = sqrt(prod(h.Focus));
                  ax = axes('Parent',p);
                  hplot = bodeplot(ax,S.Data,S.Fit,bopt);
               case 3
                  % Plot scalar or full j*G
                  bopt.PhaseWrapping = 'on';
                  if Repeat && h.FullDG(2)
                     % Full jG
                     MagLimG = localMagLimG(getPeakGain(S.Data));
                     bopt.YlimMode = {'manual'};
                     bopt.YLim = repmat({MagLimG,[-181 181]},[1 size(S.Fit,1)]);
                     bopt.Title.String = getString(message('Robust:design:dgview9',S.Block,S.Name));
                     ax = axes('Parent',p);
                     hplot = bodeplot(ax,S.Data,S.Fit,bopt);
                  else
                     if Repeat
                        % Diagonal G for repeated block
                        n = size(S.Fit,1);
                     else
                        % jG for scalar blocks
                        n = numel(S);
                     end
                     nc = round(sqrt(n));
                     nr = ceil(n/nc);
                     for ct=1:n
                        ax = subplot(nr,nc,ct,'Parent',p); %#ok<*AGROW>
                        if Repeat
                           bopt.Title.String = getString(message('Robust:design:dgview10',ct,ct,S.Block,S.Name));
                           jg = S.Data(ct,ct);  jgfit = S.Fit(ct,ct);
                        else
                           bopt.Title.String = getString(message('Robust:design:dgview9',S(ct).Block,S(ct).Name));
                           jg = S(ct).Data;  jgfit = S(ct).Fit;
                        end
                        MagLimG = localMagLimG(getPeakGain(jg));
                        bopt.YlimMode = {'manual'};
                        bopt.YLim = {MagLimG,[-181 181]};
                        hplot(ct,1) = bodeplot(ax,jg,jgfit,bopt);
                     end
                  end
            end
            legend(ax,'Data','Fit')
            h.UI(iGroup).Tabs(iTab).Plots = hplot;
         end
      end
      
   end
   
end


%-------------------------------------

function localUpdateTab(tg,h)
% Update tab content
iGroup = find(tg==[h.UI.TabGroup]);
iTab = find(tg.SelectedTab==tg.Children);
tg = h.UI(iGroup);
% Turn off visibility to speed up resize
set(cat(1,tg.Tabs.Plots),'Visible','off')
% Populate tab if necessary
h.populateTab(iGroup,iTab)
% Make tab content visible
set(tg.Tabs(iTab).Plots,'Visible','on')
end

function MagLimG = localMagLimG(gpeak)
if gpeak==0
   MagLimG = [-20,20];
else
   peakdb = 10 * ceil(2*log10(gpeak));
   MagLimG = [peakdb-100,peakdb];
end
end