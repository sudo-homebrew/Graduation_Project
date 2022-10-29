function [sys, x0, str, ts] = sfun_multiplot(t,x,u,flag,tmin,tmax,...
   ymin,ymax,Tsample,titlestr,vname)
%SFUN_MULTIPLOT S-function that acts as an TIME-Y scope using MATLAB 
%   plotting functions.  This file is designed to be used
%   with the Simulink Multiplot block in the Robust Control Toolbox.
%   It draws a line from the previous input value to the current input
%   value.  Subsequent simulation runs are plotted on the same axes.
%
%   Signals are plotted in a Matlab figure window.  If the input signal
%   is a vector, then the figure is divided into subplots, one for
%   each component of the vector signal.  Subsequent simulations add
%   lines to the axes.  The current simulation is plotted in red, while
%   previous simulations are blue.  The block acts as a "hold-on, subplotter."
%  
%   There are three toolbar buttons: "Clear", "Save", and "Autoscale".  The 
%   "Clear" button clears all lines in all axes.  The "Save" button saves all 
%   simulation to the workspace in a struct array, following the behavior of a 
%   "To Workspace" block, using the "Structure, With Time" save format. The
%   "Autoscale" button automically scales the X and Y axes according to the
%   min and max values of the signals.
%
%   See also SFUNXY, SFUNXYS, LORENZS.

%   Copyright 2006-2015 The MathWorks, Inc.
%   Andy Packard, Gary Balas 1-06-06.

blockHandle = gcbh;
% if ~isequal(flag,'Clear')
%    IsValidBlock(blockHandle, flag);
% end

if isa(flag,'double')
   % Callback from S-fcn
   %    blockHandle is the S-Fcn block
   switch flag
      case 0
        [sys,x0,str,ts] = mdlInitializeSizes(tmin,tmax,ymin,ymax,Tsample);
        LOCALSetBlockCallbacks(blockHandle);
      case 2
         sys = mdlUpdate(t,x,u,flag,tmin,tmax,ymin,ymax,titlestr,vname,...
                   blockHandle);
      case { 3, 9 }  % Unused flags
         sys = [];
      otherwise
         error('Unhandled flag: %d', flag); % sprintf-like call to ERROR
   end
elseif isa(flag,'char')
   % Callback from the SubSystem wrapped around S-Fcn
   %    blockHandle is the SubSystem's handle
   switch flag   
      case 'Start' % Start, from StartFcn callback
         LOCALBlockStartFcn(blockHandle)
      case 'Stop' % Stop, from StopFcn callback
         LOCALBlockStopFcn(blockHandle)
      case 'NameChange' % NameChange, from NameChangeFcn callback
         LOCALBlockNameChangeFcn(blockHandle)
      case 'Clear'  % Clear, from toolbar callback
         if isa(tmin,'double');
            blockHandle = tmin;
         else isa(tmin,'char');
            blockHandle = get_param(tmin,'Handle');
         end
         LOCALClear([],[],LOCALGetFigHan(blockHandle));
      case 'Save'  % Save, from toolbar callback
         if isa(tmin,'double');
            blockHandle = tmin;
         else isa(tmin,'char');
            blockHandle = get_param(tmin,'Handle');
         end
         LOCALSave([],[],LOCALGetFigHan(blockHandle));
      case 'AutoScale'  % Autoscale, from toolbar callback
         if isa(tmin,'double');
            blockHandle = tmin;
         else isa(tmin,'char');
            blockHandle = get_param(tmin,'Handle');
         end
         LOCALAutoscale([],[],LOCALGetFigHan(blockHandle));           
      case { 'CopyBlock'} % from CopyFcn
         LOCALBlockCopyFcn(blockHandle)
      case { 'LoadBlock' } % LoadFcn callbacks
         %LOCALBlockLoadFcn(blockHandle)
      case 'DeleteBlock'  % DeleteBlock, from DeleteFcn callback
         LOCALBlockDeleteFcn(blockHandle)
      case 'MaskInitialize'
         %LOCALMaskInitialize
         % do nothing for now
%       case 'DeleteFigure'
%          LOCALFigureDeleteFcn
      case 'UpdateAxes'
          LOCALUpdateAxes(blockHandle)
      otherwise % Unexpected flags
         error('Unhandled flag: ''%s''', flag);
   end
end


%==========================================================================
function [sys,x0,str,ts] = mdlInitializeSizes(tmin,tmax,~,~,Tsamp)
if ~isscalar(tmin)
   ctrlMsgUtils.error('Robust:simulink:MultiPlot2')
end
if ~isscalar(tmax)
   ctrlMsgUtils.error('Robust:simulink:MultiPlot3')
end
% XXX: need to add checking for ymin and ymax
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = -1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [Tsamp 0];


%=============================================================================
function sys = mdlUpdate(t,x,u,flag,tmin,tmax,ymin,ymax,...
   titlestr,vname,SFcnBlockH) %#ok<INUSL>
sys = [];
FHan = LOCALGetFigHan(SFcnBlockH);
if ~ishandle(FHan),
   return
end
ud = getappdata(FHan,'RCASTAppData');
nu = ud.nu;
lidx = ud.simcnt;
decimN = 19;

Nidx = (ud.currentIdx-1)/decimN;
if floor(Nidx)==ceil(Nidx) && ud.currentIdx>decimN
   % idxUse = ud.currentIdx-decimN:ud.currentIdx;
   idxUse = 1:ud.currentIdx;  % workaround deprecated EraseMode
   x_data = [ud.linfo(lidx).XData(idxUse) t];   % only last decimN points
   y_data = [ud.linfo(lidx).YData(:,idxUse) u]; % only last decimN points
   % XXX, LOCALupdateLimitsTitle(FHan,tmin,tmax,ymin,ymax,titlestr)
   set(FHan,'HandleVisibility','on')
   for i=1:nu
      set(ud.linfo(lidx).LineH(i),'Xdata',x_data,'Ydata',y_data(i,:));
   end
   set(FHan,'HandleVisibility','off')
   drawnow
end
% update the X/Y stored data points
if ud.currentIdx == length(ud.linfo(lidx).XData)
   ud.linfo(lidx).XData = [ud.linfo(lidx).XData zeros(1,ud.bufferLength)];
   ud.linfo(lidx).YData = [ud.linfo(lidx).YData zeros(nu,ud.bufferLength)];
end
ud.currentIdx = ud.currentIdx + 1;
ud.linfo(lidx).XData(ud.currentIdx) = t;
ud.linfo(lidx).YData(:,ud.currentIdx) = u;
setappdata(FHan,'RCASTAppData',ud);
% this is wherre we need to get the data.

%=============================================================================
function LOCALUpdateAxes(blockHandle)
MV = get_param(blockHandle,'MaskValues');
tmin = LocalEvaluate(MV{1},bdroot(blockHandle));
tmax = LocalEvaluate(MV{2},bdroot(blockHandle));
ymin = LocalEvaluate(MV{3},bdroot(blockHandle));
ymax = LocalEvaluate(MV{4},bdroot(blockHandle));
titlestr = MV{6};
FHan = LOCALGetFigHan(blockHandle);
if ~(FHan==-1)
   LOCALupdateLimitsTitle(FHan,tmin,tmax,ymin,ymax,titlestr)
end


%------------------------------------------------------------
function  LOCALupdateLimitsTitle(FHan,tmin,tmax,ymin,ymax,titlestr)
ud = getappdata(FHan,'RCASTAppData');
nu = ud.nu;
set(FHan,'HandleVisibility','on')
if isa(ymin,'double') && isa(ymax,'double')
   if length(ymin)==length(ymax) && length(ymax)==nu
      aymin = ymin;
      aymax = ymax;
   elseif length(ymin)==nu && length(ymax)==1
      aymin = ymin;
      aymax = ymax*ones(nu,1);
   elseif length(ymax)==nu && length(ymin)==1
      aymin = ymin*ones(nu,1);
      aymax = ymax;
   else
      aymin = ymin(1)*ones(nu,1);
      aymax = ymax(1)*ones(nu,1);
   end
else
   ctrlMsgUtils.error('Robust:simulink:MultiPlot4')
end

for i=1:nu
   set(ud.axh(i),'Xlim',[tmin tmax],'Ylim',[aymin(i) aymax(i)]);
end
set(FHan,'HandleVisibility','off')
if isa(titlestr,'char')
   set(ud.XYTitle,'String',titlestr);
else
   set(ud.XYTitle,'String','Y vs Time Plot');
end
drawnow



%=============================================================================
function LOCALBlockStartFcn(SubSysBlockH)
cpd = get(SubSysBlockH,'CompiledPortDimensions');
if size(cpd.Inport,1)==1 && cpd.Inport(1,1)==1
   nu = cpd.Inport(1,2);
else
   ctrlMsgUtils.error('Robust:simulink:MultiPlot5')
end
% porthand = get(block,'PortHandles');
% nu = get(porthand.Inport,'CompiledPortWidth');
inames = get(SubSysBlockH,'InputSignalNames');
YAxisLabel = strseq('Input ',1:nu);
if ~isempty(inames{:})            % some of the signals have names
   inames = inames{1};
   if isequal(inames(1),'<')
      inames = inames(2:end);
   end
   if isequal(inames(end),'>')
      inames = inames(1:end-1);
   end
   nidx = strfind(inames,',');
   if length(nidx)==nu-1       % same # of signals names as inputs
      nidx = [0 nidx length(inames)+1];
      for ii=1:nu
         YAxisLabel{ii} = deblank(inames(nidx(ii)+1:nidx(ii+1)-1));
      end
   end      
end
Fhan = LOCALGetFigHan(SubSysBlockH);
if ~ishandle(Fhan)
   Fhan = LOCALcreateFig(SubSysBlockH,nu,YAxisLabel);
end
LOCALUpdateAxes(SubSysBlockH);
ud = getappdata(Fhan,'RCASTAppData');
if ud.nu~=nu
   delete(Fhan);
   Fhan = LOCALcreateFig(SubSysBlockH,nu,YAxisLabel);
   ud = getappdata(Fhan,'RCASTAppData');
end
lidx = ud.simcnt;
if lidx>=0
   tmp = zeros(nu,1);
   set(Fhan,'HandleVisibility','on')
   for i=1:nu
      tmp(i) = line(0,0,'Parent',ud.axh(i),'Color',[1 0 0]);
      set(ud.XYYlabel(i),'string',YAxisLabel{i});
   end
   set(Fhan,'HandleVisibility','off')
   ud.linfo(lidx+1).XData = zeros(1,ud.bufferLength);
   ud.linfo(lidx+1).YData = zeros(ud.nu,ud.bufferLength);
   ud.linfo(lidx+1).LineH = tmp;
   ncolor = length(ud.ColorPalette);
   set(ud.linfo(lidx+1).LineH,'Color',ud.ColorPalette{mod(lidx,ncolor)+1,1});
end
ud.simcnt = lidx+1;
setappdata(Fhan,'RCASTAppData',ud);
figure(Fhan);


%=============================================================================
function LOCALBlockStopFcn(SubSysBlockH)
Fhan = LOCALGetFigHan(SubSysBlockH);
if ishandle(Fhan),
    ud = getappdata(Fhan,'RCASTAppData');
    if ud.currentIdx > 0
        % Plot the data
        ud.linfo(ud.simcnt).XData = ud.linfo(ud.simcnt).XData(:,1:ud.currentIdx);
        ud.linfo(ud.simcnt).YData = ud.linfo(ud.simcnt).YData(:,1:ud.currentIdx);
        for i=1:ud.nu
            set(ud.linfo(ud.simcnt).LineH(i),...
                'Xdata',ud.linfo(ud.simcnt).XData,...
                'Ydata',ud.linfo(ud.simcnt).YData(i,:),...
                'LineStyle','-');
        end
        ud.currentIdx = 0;
    else
        % If there is not anything logged we shouldn't store the data.  This
        % is for the case where the model is compiled using the
        % model([],[],[],'compile') syntax.
        delete(ud.linfo(ud.simcnt).LineH)
        ud.linfo(ud.simcnt) = [];
        ud.simcnt = ud.simcnt - 1;        
    end
    setappdata(Fhan,'RCASTAppData',ud);
end


%=============================================================================
function LOCALBlockNameChangeFcn(SubSysblockH)
% get the figure associated with this block, if it's valid, change
% the name of the figure
FHan = LOCALGetFigHan(SubSysblockH);
if ishandle(FHan),
   set(FHan,'Name',LOCALBlockFigureTitle(SubSysblockH));
end


%=============================================================================
function LOCALBlockCopyFcn(SubSysblockH)
MV = get(SubSysblockH,'MaskValues');
bname = get(SubSysblockH,'Name');
cnt = length(bname);
while ~isempty(str2num(bname(cnt)))
   cnt = cnt - 1;
end
endnumberstring = bname(cnt+1:length(bname));
%set(SubSysblockH,'vname',['multisimout' endnumberstring]);
MV{7} = ['multisimout' endnumberstring];
set(SubSysblockH,'MaskValues',MV);
LOCALSetFigHan(SubSysblockH,-1);


%=============================================================================
%function LOCALBlockLoadFcn(SubSysblockH)
% Can't do this because opening an existing saved version of this calls
% the BlockLoadFcn which would overwrite any saved axis data.
%MV = {'0';'20';'-1';'1';'-1';'';'multisimout'};
%set(SubSysblockH,'MaskValues',MV);

%GJB 11Oct06
%LOCALSetFigHan(SubSysblockH,-1);

%=============================================================================
function LOCALBlockDeleteFcn(SubSysblockH)
FHan = LOCALGetFigHan(SubSysblockH);
if ishandle(FHan),
   delete(FHan);
   LOCALSetFigHan(SubSysblockH,-1);
end

%=============================================================================
function FHan = LOCALcreateFig(SubSysBlockH,nu,YAxisLabel)
left = 100; bottom = 100;
screenLoc = get(0,'ScreenSize');
if screenLoc(1) < 0
    left  = -screenLoc(1) + 100;
end
if screenLoc(2) < 0
    bottom = -screenLoc(2) + 100;
end
FHan = figure('Units',          'pixel',...
  'Position',       [left bottom  450 300],...
  'Name',           LOCALBlockFigureTitle(SubSysBlockH),...
  'Tag',            'MultiPlot_FIGURE',...
  'NumberTitle',    'off',...
  'IntegerHandle',  'off');
%                   'Toolbar',        'none',...
%                   'Menubar',        'none',...
set(FHan,'DeleteFcn',{@LOCALDeleteFig FHan});
set(0,'ShowHiddenHandles','on')
toolbarhan = findobj(FHan,'Tag','FigureToolBar');
set(0,'ShowHiddenHandles','off')

% Load Cdata for Clear/Save/Autoscale Plot buttons
[EraserIm,SaveIm,BinocIm]=LOCALCreateImages;

% need to add icon to button, i.e. Cfig
uipushtool(toolbarhan,'Cdata',EraserIm,...
   'ClickedCallback',{@LOCALClear FHan},...
   'TooltipString','Clear Plots');
uipushtool(toolbarhan,'Cdata',SaveIm,...
   'ClickedCallback',{@LOCALSave FHan},...
   'TooltipString','Save Plot Data to Workspace');
uipushtool(toolbarhan,'Cdata',BinocIm,...
   'ClickedCallback',{@LOCALAutoscale FHan},...
   'TooltipString','Autoscale');
ud.Block = SubSysBlockH;
axh = zeros(nu,1);
figure(FHan);
for i=1:nu
   axh(i) = subplot(nu,1,i);
   set(axh(i),'Xgrid','on','Ygrid','on');
   ylab = YAxisLabel{i};
   if isempty(ylab)
      ud.XYYlabel(i) = ylabel(['Y' int2str(i) ' Signal']);
   else
      ud.XYYlabel(i) = ylabel(ylab);
   end
end
drawnow
subplot(nu,1,nu)
   ud.XYXlabel = xlabel('Time');
subplot(nu,1,1)
   ud.XYTitle  = get(gca,'Title');
ud.simcnt   = 0;
ud.bufferLength = 1000;
ud.currentIdx = 0;
ud.linfo = struct('XData',zeros(0,1),...
   'YData',zeros(0,1),'LineH',[]);
ud.nu = nu;
ud.axh = axh;
Colors = wavepack.colordefs('Color8');
ud.ColorPalette = Colors(1:7,1);
LOCALSetFigHan(SubSysBlockH,FHan);
set(FHan,'HandleVisibility','callback');
setappdata(FHan,'RCASTAppData',ud);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function title = BlockFigureTitle(block)
% iotype = get_param(block,'iotype');
% if strcmp(iotype,'viewer')
%    title = viewertitle(block,false);
% else
%    title = get_param(block,'Name');
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function FHan = LOCALGetFigHan(blockH)
% BlockH may be from Sfcn or SubSystem
% Figure Handle is in SubSystem's USERDATA
if strcmp(get(blockH,'BlockType'),'S-Function')
   blockH = get_param(get(blockH,'Parent'),'Handle'); % get to Subs
end
FHan = get(blockH,'UserData');
if isempty(FHan)
   % In this case, there is no Figure associated with SubSystem.
   % Return -1, as that is not a valid Handle.
   FHan = -1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALSetFigHan(blockH,Fhan)
% BlockH may be from Sfcn or SubSystem
% Figure Handle should be in SubSystem's USERDATA
if strcmp(get(blockH,'BlockType'),'S-Function'),
   blockH = get_param(get(blockH,'Parent'),'Handle'); % get to SubSystem
end
set(blockH,'UserData',Fhan);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function title = LOCALBlockFigureTitle(SubSysblockH)
title = get(SubSysblockH,'Name');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALSetBlockCallbacks(SFcnblockH)
SubSysblockH = get_param(get(SFcnblockH,'Parent'),'Handle');
callbacks={
   'CopyFcn',       'sfun_multiplot([],[],[],''CopyBlock'')' ;
   'DeleteFcn',     'sfun_multiplot([],[],[],''DeleteBlock'')' ;
   'LoadFcn',       'sfun_multiplot([],[],[],''LoadBlock'')' ;
   'StartFcn',      'sfun_multiplot([],[],[],''Start'')' ;
   'StopFcn'        'sfun_multiplot([],[],[],''Stop'')'
   'NameChangeFcn', 'sfun_multiplot([],[],[],''NameChange'')' ;
   };
for i=1:length(callbacks)
   if ~strcmp(get(SubSysblockH,callbacks{i,1}),callbacks{i,2})
      % If the CallBack is incorrect, reset it.  Note that
      % even a linked block (from a Library) can have the
      % CallBacks modified.
      set(SubSysblockH,callbacks{i,1},callbacks{i,2})
   end
end
% XXX: if the block isn't linked, issue a warning, and then set the callbacks
% for the block so that it has the proper operation
% XXXX  See old code


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALClear(~,~,FHan)
ud = getappdata(FHan,'RCASTAppData');
nu = ud.nu;
lidx = ud.simcnt;
for i=1:lidx
   for j=1:ud.nu
      delete(ud.linfo(i).LineH(j));
   end
end
figure(FHan);
tmp = zeros(nu,1);
set(FHan,'HandleVisibility','on')
for i=1:nu
   subplot(nu,1,i);
   tmp(i) = line(0,0);
   set(tmp(i),'Color',[1 0 0]);
end
set(FHan,'HandleVisibility','off')
ud.simcnt   = 0;
ud.bufferLength = 1000;
ud.currentIdx = 0;
ud.linfo    = struct('XData',zeros(1,ud.bufferLength),...
   'YData',zeros(nu,ud.bufferLength),'LineH',tmp);
% Associate the figure with the block, and set the figure's UserData.
LOCALSetFigHan(ud.Block,FHan);
set(FHan,'HandleVisibility','callback');
setappdata(FHan,'RCASTAppData',ud);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALSave(~,~,FHan)
ud = getappdata(FHan,'RCASTAppData');
for k=1:length(ud.linfo)
   S(k,1).time = ud.linfo(k).XData'; % Npts-by-1
   sig.values = ud.linfo(k).YData'; % Npts-by-Nu
   sig.dimensions = ud.nu;
   sig.label = '';
   S(k,1).signals = sig;
   S(k,1).blockName = get(ud.Block,'Name');
end
tmp = get(ud.Block,'MaskValues');
try
   assignin('base',tmp{7},S);
catch  %#ok<CTCH>
   ctrlMsgUtils.warning('Robust:simulink:MultiPlot1')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALAutoscale(~,~,FHan)
ud = getappdata(FHan,'RCASTAppData');
nu = ud.nu;
axh = ud.axh;
lidx = ud.simcnt;

% Compute max/min X-values
% (set same X-axis limits for all plots)
xmin = inf;
xmax = -inf;
for j=1:lidx
   tmpmax = max(ud.linfo(j).XData);
   if tmpmax>xmax
      xmax = tmpmax;
   end
   tmpmin = min(ud.linfo(j).XData);
   if tmpmin<xmin
      xmin = tmpmin;
   end
end

% Compute max/min Y-values for i^th axis
% and set axis limits.
for i=1:nu
   ymin = inf;
   ymax = -inf;
   for j=1:lidx
      tmpmax = max(ud.linfo(j).YData(i,:));
      if tmpmax>ymax
         ymax = tmpmax;
      end
      tmpmin = min(ud.linfo(j).YData(i,:));
      if tmpmin<ymin
         ymin = tmpmin;
      end
   end
   
   % Update Y-axis limits
   ax = [xmin xmax ymin ymax];
   if all(~isinf(ax))
      axis(axh(i),ax);
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LOCALDeleteFig(~,~,FigHan)
ud = getappdata(FigHan,'RCASTAppData');
LOCALSetFigHan(ud.Block,-1)


% function IsValidBlock(block, flag)
% if strcmp(get_param(block,'BlockType'),'S-Function'),
%   thisBlock = get_param(block,'Parent');
% else
%   thisBlock = block;
% end
% if(~strcmp(flag,'DeleteFigure'))
%   if(~strcmp(get_param(thisBlock,'MaskType'), 'HoldOnSubPlotter'))
%     error('Invalid block')
%   end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [EraserIm,SaveIm,BinocIm]=LOCALCreateImages

% Eraser Image Setup
Cfig = ones(16,16);
Cfig(:,1)   = 0;
Cfig(1,:)   = 0;
Cfig(end,:) = 0;
Cfig(:,end) = 0;
Cfig = repmat(Cfig,[1 1 3]);


%--------------Draw Eraser
Efig = Cfig;

% Eraser Outline
Efig(13,3:8,1:3) = 0;
Efig(10,3:8,1:3) = 0;
Efig(10:13,3,1:3) = 0;
Efig(10:13,8,1:3) = 0;
Efig(4,9:14,1:3) = 0;
Efig(4:7,14,1:3) = 0;
for i1=1:5
   Efig(10-i1,3+i1,1:3)=0;
   Efig(10-i1,8+i1,1:3)=0;
   Efig(13-i1,8+i1,1:3)=0;
end

% Eraser Fill
Ecolor = ones(2,4,3);
Ecolor(:,:,2) = 0.5;
Efig(11:12,4:7,1:3) = Ecolor;

Ecolor1 = ones(1,4,3);
Ecolor1(:,:,2) = 0.5;
Ecolor2 = ones(2,1,3);
Ecolor2(:,:,2) = 0.5;
for i1=1:5
   Efig(10-i1,(4:7)+i1,1:3) = Ecolor1;
   Efig((11:12)-i1,8+i1,1:3) = Ecolor2;
end

%--------------Draw Matlab Prompt
Pfig = Cfig;

% Draw Prompt Double Arrows
for i1=1:5
   Pfig(3+i1,4+i1,1:3) = 0;
   Pfig(14-i1,4+i1,1:3) = 0;
   Pfig(3+i1,8+i1,1:3) = 0;
   Pfig(14-i1,8+i1,1:3) = 0;
end

%--------------Draw Binocular Image
% This is the binocular image used in the simulink "Scope" block
% XXX-Perhaps there is any easy way to load their *.mat file? It
% is located at: matlab\toolbox\simulink\simulink\scpbarbmp.mat
Bfig(:,:,1) =[ ...
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     1     0   NaN   NaN     0     1     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN     0     0     0     0     0     0     0     0     0     0   NaN   NaN   NaN;
   NaN   NaN   NaN     0     1     0     0     0     0     0     1     0     0   NaN   NaN   NaN;
   NaN   NaN     0     0     0     0     0     0     0     0     0     0     0     0   NaN   NaN;
   NaN     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0   NaN;
   0     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0     0;
   0     0     0     1     0     0     0     0     0     0     0     1     0     0     0     0;
   0     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0];
Bfig(:,:,2) =[ ...
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     1     0   NaN   NaN     0     1     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN     0     0     0     0     0     0     0     0     0     0   NaN   NaN   NaN;
   NaN   NaN   NaN     0     1     0     0     0     0     0     1     0     0   NaN   NaN   NaN;
   NaN   NaN     0     0     0     0     0     0     0     0     0     0     0     0   NaN   NaN;
   NaN     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0   NaN;
   0     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0     0;
   0     0     0     1     0     0     0     0     0     0     0     1     0     0     0     0;
   0     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0];
Bfig(:,:,3) = [...
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     1     0   NaN   NaN     0     1     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN   NaN     0     0     0   NaN   NaN     0     0     0   NaN   NaN   NaN   NaN;
   NaN   NaN   NaN     0     0     0     0     0     0     0     0     0     0   NaN   NaN   NaN;
   NaN   NaN   NaN     0     1     0     0     0     0     0     1     0     0   NaN   NaN   NaN;
   NaN   NaN     0     0     0     0     0     0     0     0     0     0     0     0   NaN   NaN;
   NaN     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0   NaN;
   0     0     0     1     0     0     0   NaN     0     0     0     1     0     0     0     0;
   0     0     0     1     0     0     0     0     0     0     0     1     0     0     0     0;
   0     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     1     0     0     0   NaN   NaN   NaN   NaN     0     0     1     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0;
   0     0     0     0     0     0   NaN   NaN   NaN   NaN     0     0     0     0     0     0];

EraserIm = Efig;
SaveIm = Pfig;
BinocIm = Bfig;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Value = LocalEvaluate(VarName,Model)
% Evaluates model variable in appropriate workspace.
if isempty(VarName)
   Value = [];
else
   try
      Value = evalin('base',VarName);
   catch %#ok<*CTCH>
      try
         ModelWS = get_param(Model,'ModelWorkspace');
         Value = ModelWS.evalin(VarName);
      catch
         Value = [];
      end
   end
end
