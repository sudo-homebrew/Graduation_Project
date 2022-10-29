classdef ServiceParser < handle
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    
    properties (Access = private)
        %dirsToLook - Cell array of directories to search for the given srv
        DirsToLook
        
        %serviceType - Represents the service package name
        ServicePackageName
        
        %service - Holds the message file name.
        ServiceFileName
        
        %SpecialMap - Represents the special map which contains
        %data structure for special datatypes.
        SpecialMap
    end
    
    properties (SetAccess = private, GetAccess = public)
        %Input service given
        Srv
    end
    
    methods
        
        function obj = ServiceParser(srv, dirsToLook, specialMap)
            %ServiceParser constructor constructs the service parser object.
            
            if (nargin < 3)
                specialMap = containers.Map;
            end
            obj.SpecialMap = specialMap;
            obj.Srv = convertStringsToChars(srv);
            try
                [obj.ServicePackageName,obj.ServiceFileName,obj.DirsToLook] = ...
                    ros.internal.utilities.messageConstructor(srv,dirsToLook);
            catch ex
                if isequal(ex.identifier,'ros:utilities:messageparser:InvalidMessageType')
                    newEx = MException(message(...
                        'ros:utilities:serviceparser:InvalidServiceType',obj.Srv));
                    throwAsCaller(newEx);
                else
                    throwAsCaller(ex);
                end
            end
            
        end
        
        function [serviceDefinitionRequest,serviceDefinitionResponse] = ...
                getServiceDefinition(obj)
            %getServiceDefinition returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location for its request and response.
            %
            %   serviceDefinition = getServiceDefinition(parser) returns the
            %   data structure which contains FilePath, MessageType and msgFields
            %   of request and response fields of a service message.
            %
            %   Example:
            %      % ServicePackage - 'diagnostic_msgs'
            %      % ServiceFileName - 'AddDiagnostics'
            %      srvType = 'diagnostic_msgs/AddDiagnostics';
            %
            %      % srvPath should be the location where ServicePackage
            %      % exists.
            %      srvPath = {fullfile(matlabroot, 'sys', 'ros1', computer('arch'), 'ros1', 'share')};
            %      parser = ros.internal.ServiceParser(srvType, srvPath);
            %      srvInfo = getServiceDefinition(parser);
            %
            %      Here srvPath should be the location, where
            %      ServicePackage exists
            
            % Creating an empty list for Detecting Circular Dependency
            CirDependList={};
            
            % Using helper function to run main tasks
            [serviceDefinitionRequest,serviceDefinitionResponse] = ...
                getServiceDefinitionHelper(obj,CirDependList);
        end
    end
    
    methods (Access = private)
        
        function [serviceDefinitionRequest,serviceDefinitionResponse] = ...
                getServiceDefinitionHelper(obj,CirDependList)
            
            %getServiceDefinitionHelper returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location.
            
            %get file path of the service file.
            try
                filePath = ros.internal.utilities.locateMessage(...
                    obj.ServicePackageName,obj.ServiceFileName,obj.DirsToLook,'srv');
            catch ex
                if isequal(ex.identifier,'ros:utilities:messageparser:MessageNotFound')
                    newEx = MException(message(...
                        'ros:utilities:serviceparser:ServiceNotFound',obj.ServiceFileName));
                    throwAsCaller(newEx);
                elseif isequal(ex.identifier,'ros:utilities:messageparser:InvalidMessagePackage')
                    newEx = MException(message(...
                        'ros:utilities:serviceparser:InvalidServicePackage',obj.ServicePackageName));
                    throwAsCaller(newEx);
                else
                    throwAsCaller(ex);
                end
            end
            
            %call MessageParser and get contents of file.
            parser = ros.internal.MessageParser(obj.Srv,obj.DirsToLook,obj.SpecialMap);
            contentsOfFile = getMessageFileContentsWithoutComments(parser,filePath);
            
            %split the contents of file into two parts request and response
            %   splitting them by '---'.
            [contentsOfFileForRequest, contentsOfFileForResponse] = ...
                getRequestAndResponseContents(obj,contentsOfFile);
            
            %get the data structure for request part of a service file and
            %   add the string 'Request' to MessageType.
            serviceDefinitionRequest = ...
                getContentsOfServiceDefinition(obj,parser,filePath,contentsOfFileForRequest,CirDependList);
            serviceDefinitionRequest.MessageType = ...
                [serviceDefinitionRequest.MessageType 'Request'];
            
            %get the data structure for response part of a service file and
            %   add the string 'Response' to MessageType.
            serviceDefinitionResponse = ...
                getContentsOfServiceDefinition(obj,parser,filePath,contentsOfFileForResponse,CirDependList);
            serviceDefinitionResponse.MessageType = ...
                [serviceDefinitionResponse.MessageType 'Response'];
        end
        
        function [contentsOfFileForRequest, contentsOfFileForResponse] = ...
                getRequestAndResponseContents(obj,contentsOfFile) %#ok<INUSL>
            %getRequestAndResponseContents returns two data structures
            %   request and response.
            
            iterator = find(ismember(contentsOfFile,"---"));
            if isequal(numel(iterator),1)
                
                %split contents of file into two parts separated by '---'.
                contentsOfFileForRequest = contentsOfFile(1:iterator-1);
                contentsOfFileForResponse = contentsOfFile(iterator+1:end);
            else
                                     
                contentsOfFileForRequest = contentsOfFile(1:end);
                contentsOfFileForResponse = []; 
            end
        end
        
        function serviceDefinition = getContentsOfServiceDefinition(~,parser,filePath,...
                contentsOfFile,CirDependList)
            %getContentsOfServiceDefinition returns the data structure for
            %   the given contents of file.
            
            [contentsOfFile,constantValue,defaultValue,varsize,strlen] = ...
                getUpdatedMessageFileContents(parser,contentsOfFile);
            
            serviceDefinition = getDataStructure(parser,filePath,...
                contentsOfFile,constantValue,defaultValue,...
                varsize,strlen,CirDependList);
        end
        
    end
    
end

% LocalWords:  dirs srv messageparser serviceparser
