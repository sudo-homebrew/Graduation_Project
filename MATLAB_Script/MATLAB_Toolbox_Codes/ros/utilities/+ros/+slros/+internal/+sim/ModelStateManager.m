classdef ModelStateManager
%This class is for internal use only. It may be removed in the future.

%  ModelStateManager manages the ModelState for several Simulink models.
%  It does this by maintaining a persistent map from model name to
%  ModelState.
%
%  Note that all the methods are static.
%
%  See also: sim.ModelState

%  Copyright 2014-2018 The MathWorks, Inc.


    methods(Static)
        function map = getInstance()
            persistent mapInstance
            mlock;

            if isempty(mapInstance)
                mapInstance = containers.Map;
            end
            map = mapInstance;
        end
    end

    methods(Static)

        function out = hasState(modelName)
            validateattributes(modelName, {'char'}, {'nonempty'});

            map = ros.slros.internal.sim.ModelStateManager.getInstance;
            out = isKey(map, modelName);
        end

        function out = getAllModelNames()
            map = ros.slros.internal.sim.ModelStateManager.getInstance;
            out = keys(map);
        end

        function clearAll()
            map = ros.slros.internal.sim.ModelStateManager.getInstance;
            map.remove(keys(map));
        end

        function clearState(modelName)
            validateattributes(modelName, {'char'}, {'nonempty'});

            map = ros.slros.internal.sim.ModelStateManager.getInstance;
            if isKey(map, modelName)
                map.remove(modelName);
            end
        end

        function state = getState(modelName, flag)
            validateattributes(modelName, {'char'}, {'nonempty'});
            if ~exist('flag', 'var')
                flag = 'check';
            end
            validatestring(flag,{'create', 'check'});

            map = ros.slros.internal.sim.ModelStateManager.getInstance;

            if isKey(map, modelName)
                state = map(modelName);
            elseif strcmpi(flag, 'create')
                state = ros.slros.internal.sim.ModelState;
                map(modelName) = state; %#ok<NASGU>
            else
                state = ros.slros.internal.sim.ModelState.empty;
            end
        end

    end

end
