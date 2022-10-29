classdef ModelState < handle
%This class is for internal use only. It may be removed in the future.

%  ModelState holds information about a ROS model that is needed for
%  simulation. It maintains a reference count, but it is responsibility
%  of the caller to clear or delete this instance of ModelState when the
%  reference count goes to 0 (i.e., nodeHasReferrers() is false).
%
%  Note: ModelState has a ROSNode property, which is a handle to a
%  ros.Node. It is not possible to explicitly delete a Node
%  object (its destructor is private). The only way to delete it is to
%  remove all references, hence it is important to ensure that this
%  instance of ModelState is properly cleared or deleted.
%
%  See also: sim.ModelStateManager

%  Copyright 2014-2020 The MathWorks, Inc.

    properties
        % ROSNode - a handle to a ros.Node.
        ROSNode
    end

    properties (Dependent, Hidden)
        ROSBlockType
    end
    properties(SetAccess=private)
        ROSNodeRefCount = 0
    end

    methods
        function set.ROSNode(obj, node)
            assert((isa(node, 'ros.Node') || isa(node, 'ros2node') || isa(node, 'ros1node')) && numel(node) <= 1);
            obj.ROSNode = node;
        end

        function incrNodeRefCount(obj)
            obj.ROSNodeRefCount =  obj.ROSNodeRefCount + 1;
        end

        function decrNodeRefCount(obj)
            obj.ROSNodeRefCount =  obj.ROSNodeRefCount - 1;
        end

        function out = nodeHasReferrers(obj)
            out = obj.ROSNodeRefCount > 0;
        end

    end
end
