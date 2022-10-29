function index = findProp(prop,varargin)
%findProp - Find a property in a list of Name-value pairs and return it's
%index.
%
% This is an internal function and may be removed or modified in the future
%
% index = findProp(prop,varargin) looks for the property, prop, in the list
% of Name-value pairs, varargin. If found, the index is returned as the
% position at which Name was found. If not found, index is returned as
% numel(varargin) + 1
%
% Example: find the property 'myProp' in a list
% index = fusion.internal.findProp('myProp',...
%   'notMyProp', 4, 'myProp', 2)

%   Copyright 2018 The MathWorks, Inc.

%#codegen

index = matlabshared.smoothers.internal.findProp(prop,varargin{:});