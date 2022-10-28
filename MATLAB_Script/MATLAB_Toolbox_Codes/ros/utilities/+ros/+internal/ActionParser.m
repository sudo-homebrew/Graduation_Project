classdef ActionParser < handle
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    
    properties (Access = private)
        %dirsToLook - Cell array of directories to search for the given action
        DirsToLook
        
        %serviceType - Represents the service package name
        ActionPackageName
        
        %service - Holds the message file name.
        ActionFileName
        
        %SpecialMap - Represents the special map which contains
        %data structure for special datatypes.
        SpecialMap
    end
    
    properties (SetAccess = private, GetAccess = public)
        %Input service given
        Action
    end
    
    methods
        
        function obj = ActionParser(action, dirsToLook, specialMap)
            %ActionParser constructor constructs the service parser object.
            
            if (nargin < 3)
                specialMap = containers.Map;
            end
            obj.SpecialMap = specialMap;
            obj.Action = convertStringsToChars(action);
            try
                [obj.ActionPackageName,obj.ActionFileName,obj.DirsToLook] = ...
                    ros.internal.utilities.messageConstructor(action,dirsToLook);
            catch ex
                if isequal(ex.identifier,'ros:utilities:messageparser:InvalidMessageType')
                    newEx = MException(message(...
                        'ros:utilities:actionparser:InvalidActionType',obj.Action));
                    throwAsCaller(newEx);
                else
                    throwAsCaller(ex);
                end
            end
        end
        
        function [actionDefinitionGoal,actionDefinitionFeedback,actionDefinitionResult] = ...
                getActionDefinition(obj)
            %getActionDefinition returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location.
            %
            %   actionDefinition = getActionDefinition(parser) returns the
            %   data structure which contains FilePath, MessageType and msgFields
            %   for goal, result and feedback of an action message.
            %
            %   Example:
            %      % ActionPackage - 'actionlib'
            %      % ActionFileName - 'Test'
            %      actionType = 'actionlib/Test';
            %
            %      % actionPath should be the location where ActionPackage
            %      % exists.
            %      actionPath = {fullfile(matlabroot, 'sys', 'ros1', computer('arch'), 'ros1', 'share')};
            %      parser = ros.internal.ActionParser(actionType, actionPath);
            %      actionInfo = getActionDefinition(parser);
            %
            %      Here actionPath should be the location, where
            %      ActionPackage exists
            
            % Creating an empty list for Detecting Circular Dependency
            CirDependList={};
            
            % Using helper function to run main tasks
            [actionDefinitionGoal,actionDefinitionFeedback,actionDefinitionResult] = ...
                getActionDefinitionHelper(obj,CirDependList);
        end
        
    end
    
    methods (Access = private)
        
        function [actionDefinitionGoal,actionDefinitionResult,actionDefinitionFeedback] = ...
                getActionDefinitionHelper(obj,CirDependList)
            
            %getActionDefinition returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location.
            
            %get file path of the action file.
            try
                filePath = ros.internal.utilities.locateMessage(...
                    obj.ActionPackageName,obj.ActionFileName,obj.DirsToLook,'action');
            catch ex
                if isequal(ex.identifier,'ros:utilities:messageparser:MessageNotFound')
                    newEx = MException(message(...
                        'ros:utilities:actionparser:ActionNotFound',obj.ActionFileName));
                    throwAsCaller(newEx);
                elseif isequal(ex.identifier,'ros:utilities:messageparser:InvalidMessagePackage')
                    newEx = MException(message(...
                        'ros:utilities:actionparser:InvalidActionPackage',obj.ActionPackageName));
                    throwAsCaller(newEx);
                else
                    throwAsCaller(ex);
                end
            end
            
            %call MessageParser and get contents of file.
            parser = ros.internal.MessageParser(obj.Action,obj.DirsToLook,obj.SpecialMap);
            contentsOfFile = getMessageFileContentsWithoutComments(parser,filePath);
            
            %split the contents of file into three parts goal, result and
            %   feedback splitting them by '---'.
            [contentsOfFileForGoal, contentsOfFileForResult, contentsOfFileForFeedback] = ...
                getGoalResultFeedbackContents(obj,contentsOfFile);
            
            %get the data structure for goal part of an action file and
            %   add the string 'Goal' to MessageType.
            actionDefinitionGoal = ...
                getContentsOfActionDefinition(obj,parser,filePath,contentsOfFileForGoal,CirDependList);
            actionDefinitionGoal.MessageType = ...
                [actionDefinitionGoal.MessageType 'Goal'];
            
            %get the data structure for result part of an action file and
            %   add the string 'Result' to MessageType.
            actionDefinitionResult = ...
                getContentsOfActionDefinition(obj,parser,filePath,contentsOfFileForResult,CirDependList);
            actionDefinitionResult.MessageType = ...
                [actionDefinitionResult.MessageType 'Result'];
            
            %get the data structure for feedback part of an action file and
            %   add the string 'Feedback' to MessageType.
            actionDefinitionFeedback = ...
                getContentsOfActionDefinition(obj,parser,filePath,contentsOfFileForFeedback,CirDependList);
            actionDefinitionFeedback.MessageType = ...
                [actionDefinitionFeedback.MessageType 'Feedback'];
        end
        
        function [contentsOfFileForGoal, contentsOfFileForResult, contentsOfFileForFeedback] = ...
                getGoalResultFeedbackContents(obj,contentsOfFile)
            %getGoalFeedbackActionContents returns three data structures
            %   goal, result and feedback.
            
            iterator = find(ismember(contentsOfFile,"---"));
            
            if isequal(numel(iterator),2)
                
                %split contents of file into two parts separated by '---'.
                contentsOfFileForGoal = contentsOfFile(1:iterator(1)-1);
                contentsOfFileForResult = contentsOfFile(iterator(1)+1:iterator(2)-1);
                contentsOfFileForFeedback = contentsOfFile(iterator(2)+1:end);
            else
                error(message(...
                    'ros:utilities:actionparser:InvalidActionDefinition',obj.Action))
            end
        end
        
        function actionDefinition = getContentsOfActionDefinition(~,parser,filePath,...
                contentsOfFile,CirDependList)
            %getContentsOfActionDefinition returns the data structure for
            %   the given contents of file.
            
            [contentsOfFile,constantValue,defaultValue,varsize,strlen] = ...
                getUpdatedMessageFileContents(parser,contentsOfFile);
            
            actionDefinition = getDataStructure(parser,filePath,...
                contentsOfFile,constantValue,defaultValue,...
                varsize,strlen,CirDependList);
        end
        
    end
    
end

% LocalWords:  dirs messageparser actionparser actionlib 
