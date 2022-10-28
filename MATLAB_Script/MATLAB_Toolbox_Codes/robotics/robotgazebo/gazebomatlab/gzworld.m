function output = gzworld(op)
%GZWORLD interact with Gazebo world
%   GZWORLD("reset") reset all Gazebo model configurations and Gazebo
%   simulation time
%
%   Example:
%      % Launch 'multiSensorPluginTest.world' from installed Gazebo server
%      % plugin before using following example
%
%      % Reset Gazebo model configurations
%      GZWORLD("reset")
%
%   See also gzinit, gzmodel, gzlink, gzjoint

%   Copyright 2020 The MathWorks, Inc.

    narginchk(1,1);

    op = convertStringsToChars(op);
    validateattributes(op, {'char'}, {'scalartext', 'nonempty'},...
                       'gzworld', 'operation');

    % Use validatestring to ensure string and char inputs to 'operation'
    % are supported.
    supportedOperations = {'reset'};
    try
        operation = validatestring(op, supportedOperations, 'gzworld', 'operation');
    catch
        error(message('robotics:robotgazebo:gzsupport:InvalidOperation', op, ...
                      strjoin(supportedOperations,'","')));
    end

    switch(operation)
      case 'reset'
        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.world.resetWorld;
        else
            output = robotics.gazebo.internal.MATLABInterface.world.resetWorld;
        end
    end

end
