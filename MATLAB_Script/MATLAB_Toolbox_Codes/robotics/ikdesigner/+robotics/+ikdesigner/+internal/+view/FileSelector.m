classdef FileSelector < handle
%This class if for internal use only and may be removed in a future release

%FileSelector Object for accessing the File I/O views
%   This object mostly provides wrapper methods for the existing uigetfile
%   and uiputfile methods. They add some extra functionality surround the
%   supporting display updates.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = immutable)
        AppWindow
    end

    methods
        function obj = FileSelector(appWindow)
            %FileSelector Constructor

            obj.AppWindow = appWindow;
        end
        
        function [isFileSelected, filepath] = chooseFileToLoad(obj,varargin)
            %chooseFileToLoad Bring up the File I/O to load existing data
            %   The method calls File I/O with an "open" button and brings
            %   the window to the front.

            [file,folder] = uigetfile(varargin{:});
            [isFileSelected, filepath] = obj.processUIData(file, folder);
            bringToFront(obj.AppWindow);
        end
        
        function [isFileSelected, filepath] = chooseFileToSave(obj,varargin)
            %chooseFileToLoad Bring up the File I/O to save data
            %   The method calls File I/O with a "save" button and brings
            %   the window to the front.

            [file,folder] = uiputfile(varargin{:});
            [isFileSelected, filepath] = obj.processUIData(file, folder);
            bringToFront(obj.AppWindow);
        end
    end

    methods (Static, Access = private)
        function [isFileSelected, filepath] = processUIData(file, folder)
            %processUIData Process data from uigetfile or uiputfile
            %   Converts file name and path to a flag indicating whether a
            %   file is returned and a complete file path.

            if file == 0
                isFileSelected = false;
                filepath = string.empty;
            else
                isFileSelected = true;
                filepath = fullfile(folder, file);
            end

        end
    end
end
