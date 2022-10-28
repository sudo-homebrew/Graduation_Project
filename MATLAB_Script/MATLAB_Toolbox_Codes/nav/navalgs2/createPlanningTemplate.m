function createPlanningTemplate(varargin)
%createPlanningTemplate Create sample implementation for path planning interface
%   createPlanningTemplate creates a sample implementation of a path planning
%   interface deriving from the nav.StateSpace class. The function opens new
%   files in the MATLAB editor.
%
%   createPlanningTemplate(TEMPLATETYPE) creates a sample implementation
%   for the path planning interface specified by TEMPLATETYPE. Valid values
%   for TEMPLATETYPE are "StateSpace" and "StateValidator".
%
%
%   Example:
%      % Open state space template in editor
%      createPlanningTemplate
%
%      % Open state validator template in editor
%      createPlanningTemplate("StateValidator")
%
%   See also nav.StateSpace, nav.StateValidator.

%   Copyright 2019-2020 The MathWorks, Inc.

    narginchk(0,1);

    if nargin == 1
        % Ensure that input is a scalar string
        templateType = robotics.internal.validation.validateString(varargin{1}, false, ...
                                                          "createPlanningTemplate", "templateType");
    else
        templateType = "StateSpace";
    end

    % Validate that the user input is a valid template choice
    templateType = validatestring(templateType, ["StateSpace", "StateValidator"], "createPlanningTemplate", "templateType");

    switch(templateType)
      case "StateSpace"
        fileName = "StateSpaceExample";
      case "StateValidator"
        fileName = "StateValidatorExample";
    end

    % Read in template code. Use full path to avoid local versions
    % of the file from being read.
    exampleFile = fullfile(toolboxdir("nav"), "navalgs2", "+nav",...
                           "+algs", "+internal", fileName + ".m");
    fid = fopen(exampleFile);
    contents = fread(fid, "*char");
    fclose(fid);

    % Open template code in an Untitled file in the editor
    editorDoc = matlab.desktop.editor.newDocument(contents(:)');

    % Change the function name to a custom name. Replace all instances of the
    % class name with the new custom name.
    contents = regexprep(editorDoc.Text,...
                         fileName, "MyCustom" + templateType);

    % Remove the MathWorks Copyright line
    contents = regexprep(contents,...
                         "^%   Copyright.*$", "", "lineanchors", "dotexceptnewline", "once");

    % Reload the modified file contents in the editor window
    editorDoc.Text = contents;
    editorDoc.smartIndentContents;
    editorDoc.goToLine(1);
end
