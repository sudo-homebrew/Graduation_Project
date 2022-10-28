classdef FileSystem
%This class is for internal use only. It may be removed in the future.

%FileSystem Helper functions for handling the file system

%   Copyright 2014-2021 The MathWorks, Inc.

    methods (Static)
        function files = listFiles(path, filePattern, varargin)
        %listFiles List files that match a given pattern
        %   You can use the patterns that the "dir" command accepts.
        %
        %   FILES = listFiles(PATH, FILEPATTERN) returns a cell array
        %   of absolute file paths for files that are found under
        %   PATH and match the pattern FILEPATTERN.
        %
        %   FILES = listFiles(PATH, FILEPATTERN, RECURSIVE) returns a
        %   cell array of absolute file paths. If RECURSIVE is set to
        %   TRUE, it will recursively search all sub-folders as well. By
        %   default, RECURSIVE is false.
        %
        %   Examples:
        %      files = ros.internal.FileSystem.listFiles('c:\work', '*.jar')
        %      files = ros.internal.FileSystem.listFiles('./relative', '*.*')
        %      files = ros.internal.FileSystem.listFiles('./relative', '*.m')
        %
        %      % Recursively list all MATLAB Files that start with 'A'
        %      files = ros.internal.FileSystem.listFiles('./relative', 'A*.m', true)

        % Parse inputs
            defaults.recursive = false;
            args = ros.internal.FileSystem.parseListInput(defaults, path, filePattern, varargin{:});

            % Initialize output
            files = {};

            % Search recursively if specified by the user
            if args.recursive
                folders = ros.internal.FileSystem.listFolders(args.path, '*');
                for i = 1:numel(folders)
                    files = [files; ros.internal.FileSystem.listFiles(...
                        folders{i}, args.pattern, true)]; %#ok<AGROW>
                end
            end

            dirStruct = dir(fullfile(args.path, args.pattern));
            if isempty(dirStruct)
                return;
            end

            % Get indices of actual files
            fileIdx = ~(cell2mat({dirStruct.isdir}));
            fileStruct = dirStruct(fileIdx);
            files = [files; cellfun(@(x) fullfile(args.path, x), {fileStruct.name}, ...
                                    'UniformOutput', false)'];
        end

        function folders = listFolders(path, pattern, varargin)
        %listFolders List folders that match a given pattern
        %   You can use the patterns that the "dir" command accepts.
        %
        %   FOLDERS = listFolders(PATH, PATTERN) returns a cell array
        %   of absolute paths for folders that are found under
        %   PATH and match the pattern PATTERN.
        %
        %   FOLDERS = listFolders(PATH, PATTERN, RECURSIVE) returns a
        %   cell array of absolute folder paths. If RECURSIVE is set to
        %   TRUE, it will recursively search all sub-folders as well. By
        %   default, RECURSIVE is false.
        %
        %   Examples:
        %      folders = ros.internal.FileSystem.listFolders('c:\work', '*')
        %      folders = ros.internal.FileSystem.listFolders('./relative', 'A*')
        %
        %      % Recursively list all folders with "internal" in name
        %      folders = ros.internal.FileSystem.listFolders('./relative', '*internal', true)

        % Parse inputs
            defaults.recursive = false;
            args = ros.internal.FileSystem.parseListInput(defaults, path, pattern, varargin{:});

            % Initialize output
            folders = {};

            % Search recursively if specified by the user
            if args.recursive
                recFolders = ros.internal.FileSystem.listFolders(args.path, '*');
                for i = 1:numel(recFolders)
                    folders = [folders; ros.internal.FileSystem.listFolders(...
                        recFolders{i}, args.pattern, true)]; %#ok<AGROW>
                end
            end

            dirStruct = dir(fullfile(args.path, args.pattern));
            % Exclude '.' and '..' from the search
            invalidDirsIdx = ~arrayfun(@(x) strcmp(x.name, '.') || strcmp(x.name, '..'), dirStruct);
            dirStruct = dirStruct(invalidDirsIdx);
            if isempty(dirStruct)
                return;
            end

            % Get indices of actual files
            dirIdx = cell2mat({dirStruct.isdir});
            dirStruct = dirStruct(dirIdx);
            folders = [folders; cellfun(@(x) fullfile(args.path, x), {dirStruct.name}, ...
                                        'UniformOutput', false)'];
        end

        function absPath = relativeToAbsolute(relativePath)
        %relativeToAbsolute Convert relative to absolute path

            validateattributes(relativePath, {'char'}, {'nonempty'}, ...
                               'relativeToAbsolute', 'relativePath');

            % Special case to handle '.' and '..' input that Java handled
            % but the internal absolutePath utility does not (g2550523)
            if startsWith(relativePath, '.')
                relativePath = fullfile(pwd, relativePath);
            end

            absPath = char(matlab.io.internal.filesystem.absolutePath(relativePath));

            % Use relative path if there was an issue
            if ismissing(absPath)
                absPath = relativePath;
            end
        end
    end

    methods (Static, Access = private)
        function args = parseListInput(defaults, path, pattern, varargin)
        %parseListInput Parse the input to the list* functions

            persistent parser

            if isempty(parser)
                parser = inputParser;
                addRequired(parser, 'path', ...
                            @(x) validateattributes(x, {'char'}, {'nonempty'}, ...
                                                    'parseListInput', 'path'));
                addRequired(parser, 'pattern', ...
                            @(x) validateattributes(x, {'char'}, {'nonempty'}, ...
                                                    'parseListInput', 'pattern'));
                addOptional(parser, 'recursive', defaults.recursive, ...
                            @(x) validateattributes(x, {'logical', 'numeric'}, ...
                                                    {'nonempty', 'scalar'}, 'parseListInput', 'recursive'));
            end

            parse(parser, path, pattern, varargin{:});
            args.path = ros.internal.FileSystem.relativeToAbsolute(parser.Results.path);
            args.pattern = parser.Results.pattern;
            args.recursive = parser.Results.recursive;
        end
    end

end
