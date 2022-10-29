%MTIMES  Multiplication for ICSIGNAL objects

function out = mtimes(varargin)
% Copyright 2003-2004 The MathWorks, Inc.

if nargin==2 & isa(varargin{2},'icsignal')
    sys = varargin{1}*varargin{2}.System;
    out = icsignal(sys,varargin{2}.SignalList,varargin{2}.SignalDim);
elseif nargin>2 & isa(varargin{end},'icsignal')
    out = mtimes(varargin{1:end-2},varargin{end-1}*varargin{end});
end