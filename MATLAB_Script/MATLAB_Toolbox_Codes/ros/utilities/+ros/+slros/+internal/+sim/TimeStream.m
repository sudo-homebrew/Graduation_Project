classdef TimeStream < handle
%This class is for internal use only. It may be removed in the future.

%TimeStream Encapsulates an interface for getting ROS time
%   This interface is purely abstract and has to be implemented by
%   concrete subclasses.

%   Copyright 2018-2020 The MathWorks, Inc.

    methods (Abstract)
        currentTime = getTime(obj)
        %getTime Get the current time
        %   This abstract method has to be implemented by all subclasses of
        %   TimeStream.
    end

end
