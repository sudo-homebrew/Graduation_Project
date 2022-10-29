function out = populateRBTStructFromFunction(func, varargin)
    %This class is for internal use only. It may be removed in the future.
    
    %populateRBTStructFromFunction extract rigidBodyTree struct from a function generated rigidBodyTree
    %   Extract a rigidBodyTree struct from a rigidBodyTree created by a 
    %   function given by FUNC input. This helper is designed to be used to
    %   generate code for rigidBodyTree objects created using a function.
    %
    %   Example:
    %      rbt = coder.const(@feval, 'robotics.manip.internal.extractStructFromRBT', ...
    %       'importrobot', input, varargin{:});
    
    %   Copyright 2021 The MathWorks, Inc.
    
    %Call to rigidBodyTree generating function
    rbt = feval(str2func(func), varargin{:});

    %Extract struct representation of RBT with collision data
    skipCollisionData = false;
    out = robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyTreeStruct(rbt, skipCollisionData);
end