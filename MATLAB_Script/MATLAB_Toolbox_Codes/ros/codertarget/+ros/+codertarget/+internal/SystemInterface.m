classdef SystemInterface < handle
%This class is for internal use only. It may be removed in the future.

%SystemInterface Define an interface for system operations supported on Linux hardware
%
%   SystemInterface is an abstract class which
%   defines the interface methods for communicating with a hardware.
%
%   The following methods must be implemented by classes which derive from
%   this base class.
%
%   output = system(OBJ,COMMAND,SUDO) Called when a system command gets
%   executed on the hardware. SUDO is an optional argument.
%
%   putFile(obj,localFile,remoteFile) Copy locaSrc on the host computer to
%   remoteFile on the hardware.
%
%   getFile(obj,remoteFile,localFile) Copy the remoteFile on the hardware to
%   to the localFile on the host computer.
%
%   deleteFile(obj,fileName) Deletes specified file from the hardware.
%
%   d = dir(obj,fileSpec) Returns a directory listing.

%   Copyright 2015-2018 The MathWorks, Inc.

    methods (Access=public, Abstract)
        output = system(obj,command,sudo);
        putFile(obj,localFile,remoteFile);
        getFile(obj,remoteFile,localFile);
        deleteFile(obj,fileName);
        d = dir(obj,fileSpec);
    end
    
    methods (Static, Abstract)
        p = fullfile(varargin);
    end
end
