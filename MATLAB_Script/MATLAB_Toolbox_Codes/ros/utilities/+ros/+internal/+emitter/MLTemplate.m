classdef MLTemplate < handle
%This class is for internal use only. It may be removed in the future.

%MLTemplate renders data into a template and creates the file.

%Example:
%
%    m = MLTemplate;
%    m.loadFile('test.m.tmpl'); % creates the required tree
%    s = struct('name', 'abc', 'age', '29');
%    m.render(s); % renders the data creating 'test.m' file
%

%   Copyright 2019 The MathWorks, Inc.

    properties
        root;
        
        % This field decides if the render() will create a new file or it 
        % will append the contents in an existing file. The default value
        % is 'wt' which will create a new file. It can be set with
        % different values of "permission" argument in "fopen()" MATLAB API.
        outFileOperation = 'wt';
    end

    properties (Hidden)
        outFile;
    end

    methods
        function loadString(self, templatestring)
        %Takes the file and start loading all the contents in it.

            compiler = ros.internal.emitter.MLCompiler();
            self.root = compiler.compile(templatestring);
        end

        function loadFile(self, filename)
        %Takes the filename as input, reads the file and creates an
        %   empty output file.

            string = fileread(filename);
            self.outFile =  strrep(filename, '.tmpl', '');
            self.loadString(string);
        end

        function render(self, values, varargin)
        %Renders the data into the template and writes into the output file.

        %add TempSpace to values
            if ~isstruct(values)
                error(message('ros:utilities:templates:NotStruct'));
            end
            values.(ros.internal.emitter.MLTempSpace.TEMPSPACENAME) = ros.internal.emitter.MLTempSpace;

            str = self.root.render(values);
            %remove white space and empty lines
            %replace only >2 \n with 2 \n because makefiles etc may need
            %the extra line
            if nargin > 2
                strip = varargin{1};
            else
                strip = 0;
            end
            switch(strip)
              case 1, str = regexprep(str,'\r*\n[\s\r\n]*\n','\n\n'); %strip only the middle
              case 2, str = regexprep(str,'\r*\n[\s\r\n]*\n','\n'); %strip all multiple \n
            end
            if ~isempty(self.outFile)
                fp = fopen(self.outFile, self.outFileOperation);
                if fp<0
                    error(message('ros:utilities:templates:FileNotFound',self.outFile));
                else
                    fwrite(fp, str);
                    fclose(fp);
                end
            else
                disp(str);
            end
        end
    end
end
