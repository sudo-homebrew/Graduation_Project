function [sys,varargout] = ulinearize(varargin)
%ULINEARIZE  Linearize a Simulink model with Uncertain State Space blocks.
%
%  ULIN = ULINEARIZE('sys',IO) takes a Simulink model name, 'sys' and an
%  I/O object, IO, as inputs and returns a linear time-invariant uncertain
%  system ULIN. The uncertainty is specified in Uncertain State Space
%  blocks. The linearization I/O object is created with the function
%  GETLINIO or LINIO. 
%
%  For additional input options to specify linearization options, operating
%  points and simulation snapshots see the help for LINEARIZE.
%
% See Also: LINEARIZE, LINIO, OPERPOINT, USS

%  Author(s): Gary Balas
%  Copyright 2009 The MathWorks, Inc.
nargoutchk(0, 2)
% Support strings arguments
[varargin{:}] = convertStringsToChars(varargin{:});

% Check for Simulink Control Design
if ~license('test','Simulink_Control_Design')
    error('Robust:simulink:ULINEARIZELicense','The product Simulink Control Design is required to use the command ULINEARIZE.')
end

% Find the uncertain state space blocks in the Simulink model.
mdl = varargin{1};
[~,pathinfo] = ufind(mdl);
allupaths = unique(pathinfo(:,1));

Nuss = numel(allupaths);
for ct = Nuss:-1:1
    % Create the replacement structure.
    rep = struct('Specification',get_param(allupaths{ct},'USystem'),...
             'Type','Expression',...
             'ParameterNames','',...
             'ParameterValues','');
    TunedBlocks(ct) = struct('Name',allupaths(ct),...
                         'Value',rep);
end

newargs = varargin(2:end);
if Nuss > 0
    newargs = [newargs,{TunedBlocks}];
end

if nargout<=1
    sys = linearize(mdl,newargs{:});
elseif nargout==2
    [sys,varargout{1}] = linearize(mdl,newargs{:});
end