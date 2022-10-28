function out = optunit(name,value,default,string,publicflag,varargin)
%OPTUNIT Creates an options object

%   Copyright 2003-2011 The MathWorks, Inc.
superiorto('double','char');
out.Name = name;
out.Value = value;
out.Default = default;
out.String = string;
out.PublicFlag = publicflag;
out = class(out,'optunit',gsref());
