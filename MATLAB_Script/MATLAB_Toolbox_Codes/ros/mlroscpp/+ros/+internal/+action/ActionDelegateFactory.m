classdef ActionDelegateFactory < handle
    %This class is for internal use only. It may be removed in the future.
    
    %ActionDelegateFactory Singleton factory for action-related delegates
    %   This prime purpose of this factory is to allow unit tests to
    %   replace instances of action classes with mock objects.
    %
    %   This design follows the Dependency Lookup / Component Broker pattern
    %   described in http://xunitpatterns.com/Dependency%20Lookup.html.
    
    %   Copyright 2016-2020 The MathWorks, Inc.
    
    properties (Access = private)
        %DelegateMap - Map from component name to delegate object
        DelegateMap
    end
    
    methods(Static)
        function factory = getInstance()
            %getInstance Get the singleton instance of the factory
            %   Once created, the instance will stored in a persistent
            %   variable.
            
            persistent delegateFactory
            
            if isempty(delegateFactory)
                delegateFactory = ros.internal.action.ActionDelegateFactory;
            end
            
            factory = delegateFactory;
        end
    end
    
    methods
        function [delegate, component] = findDelegate(obj, compName, varargin)
            %findDelegate Find delegate for component name
            %   This call will return a new instance of the requested
            %   component. If a component of this name does not exist, the
            %   function will display an error.
            
            component = obj.DelegateMap(compName);
            if isa(component, 'function_handle')
                % If specified as a function handle, create a new instance
                delegate = feval(component, varargin{:});
            else
                % If this is already an instantiated delegate, return it
                delegate = component;
            end
        end
        
        function registerDelegateForName(obj, compName, delegate)
            %registerDelegateForName Register a delegate for a component name
            %   If DELEGATE is a function handle, it is interpreted as a class name
            %   and findDelegate will return a newly constructed instance.
            %   If DELEGATE is an object, it will be returned as-is by
            %   findDelegate.
            
            obj.DelegateMap(compName) = delegate;
        end
    end
    
    methods (Access = private)
        function obj = ActionDelegateFactory
            %ActionDelegateFactory Construct the delegate factory
            %   This constructor is private, so that all instantiations
            %   are managed by the getInstance method.
            
            obj.DelegateMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            % Register the default action delegates
            obj.registerDelegateForName('simpleClientDelegate', ...
                @ros.internal.action.SimpleActionClientDelegate);
            obj.registerDelegateForName('simpleClientParser', ...
                @ros.internal.action.SimpleActionClientParser);
            obj.registerDelegateForName('actionIntrospection', ...
                @ros.internal.action.ActionIntrospection);
        end
    end
    
end

