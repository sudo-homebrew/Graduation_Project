classdef (Hidden) RandomNumberGenerator < matlab.System

    % RandomNumberGenerator sets the random number settings in Simulink block.
    %
    % This is an internal class and may be removed or modified in a future
    % release.

    % Copyright 2021 The MathWorks, Inc.

    %#codegen

    properties(Nontunable)
        %InitialSeedSource  Select method to specify initial seed
        %   The initial seed used to generate random numbers for the
        %   tracker block using one of the methods: 'Specify seed' |
        %   'Repeatable' | 'Not repeatable'. When set to 'Specify seed',
        %   the value set in the InitialSeed property is used. When set to
        %   'Repeatable', a random initial seed is generated for the first
        %   simulation and then reused for all subsequent simulations
        %   (issuing a 'clear all' command will force a new seed to be
        %   generated). When set to 'Not repeatable', a new initial seed is
        %   used each time the simulation is run. This property is only
        %   valid in Simulink.
        %
        %   Default: 'Repeatable'
        InitialSeedSource = 'Repeatable'

        %UsedSeed  Initial seed used for simulation
        %   Holds the initial seed used by the sensor for simulation. This seed
        %   is randomly generated when the InitialSeedSource property is set to
        %   'Repeatable' or 'Not repeatable'. When the InitialSeedSource
        %   property is set to 'Specify seed', the value set in the InitialSeed
        %   property is used. This property is only valid in Simulink.
        UsedSeed = 0

        %InitialSeed  Initial seed for random number generator
        %   Specify the seed for the random number generator as a
        %   non-negative integer. The integer must be less than 2^32. This
        %   property is only valid in Simulink.
        %
        %   Default: 0
        InitialSeed(1,1) {mustBeInteger,mustBeNonnegative,mustBeLessThan(InitialSeed,4294967295)} = 0
    end

    properties(Constant, Hidden)
        InitialSeedSourceSet = matlab.system.StringSet({'Repeatable','Not repeatable','Specify seed'})
    end

    methods
        function obj = RandomNumberGenerator(varargin)
            if nargin>0
                idx = fusion.internal.findProp('InitialSeedSource',varargin{:});
                if ~strcmpi(varargin{idx+1},'Specify seed')
                    obj.InitialSeedSource = varargin{idx+1};
                    idxSeed = fusion.internal.findProp('UsedSeed',varargin{:});
                    if idxSeed>nargin
                        obj.UsedSeed = 0;
                    else
                        obj.UsedSeed = varargin{idxSeed+1};
                    end
                else
                    obj.InitialSeedSource = varargin{idx+1};
                    initSeed = fusion.internal.findProp('InitialSeed',varargin{:});
                    if initSeed>nargin
                        obj.InitialSeed = 0;
                    else
                        obj.InitialSeed = varargin{initSeed+1};
                    end
                end
            else
                setProperties(obj,varargin{:});
            end
        end

        function val = toStruct(obj)
            if ~strcmpi(obj.InitialSeedSource,'Specify seed')
                val = struct('InitialSeedSource',obj.InitialSeedSource, ...
                    'UsedSeed',obj.UsedSeed);
            else
                val = struct('InitialSeedSource',obj.InitialSeedSource, ...
                    'InitialSeed',obj.InitialSeed);
            end
        end
    end

    methods(Static)
        function val = toRandfromStruct(st)
            % Convert RandomNumberGenerator object from struct
            if ~strcmpi(st.InitialSeedSource,'Specify seed')
                val = fusion.simulink.internal.RandomNumberGenerator('InitialSeedSource', ...
                    st.InitialSeedSource, 'UsedSeed',st.UsedSeed);
            else
                val = fusion.simulink.internal.RandomNumberGenerator('InitialSeedSource', ...
                    st.InitialSeedSource, 'InitialSeed',st.InitialSeed);
            end
        end
    end

    methods(Static, Access = protected)
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = false;
            flag = flag ||((strcmpi(obj.InitialSeedSource,'Repeatable')||...
                strcmpi(obj.InitialSeedSource,'Not repeatable'))&&strcmpi(prop,'InitialSeed'));
        end

        function section = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            pUsedSeed = matlab.system.display.internal.Property(...
                'UsedSeed','IsGraphical',false,'UseClassDefault',false,...
                'Default','matlabshared.tracking.internal.SimulinkUtilities.seedManager(gcb)');

            randSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:RandomNumberGenerator','RandNumber',{'InitialSeedSource','InitialSeed',pUsedSeed});
            section = randSection;
        end
    end

    methods(Static,Hidden)
        function seed = lastInitialSeed(blkPath)
            % fusion.simulink.internal.RandomNumberGenerator Last initial seed used by
            %   Simulink block. The initial seed used to generate random
            %   numbers for the model is specified using one of the
            %   methods: 'Specify seed' | 'Repeatable' | 'Not repeatable'.
            %   When the 'Repeatable' or 'Not repeatable' methods are used,
            %   the initial seed will be randomly assigned to the tracker
            %   block. This method allows the seed last used by the tracker
            %   block to be determined.

            narginchk(0,1);

            if nargin<1
                blkPath = gcb;
            end

            % Is this inside a model reference? If so, then load it
            [loadedModels, resolvedBlkPath] = ...
                matlabshared.tracking.internal.SimulinkUtilities.loadModels(blkPath);
            mkClean = onCleanup(@()matlabshared.tracking.internal.SimulinkUtilities.closeModels(loadedModels));

            if ~strcmp(get_param(resolvedBlkPath,'BlockType'),'MATLABSystem')
                error(message('fusion:simulink:RandomNumberGenerator:mustBeMATLABSystemBlock'));
            end

            objClass = get_param(resolvedBlkPath,'System');
            sysObj = feval(objClass);
            if ~isa(sysObj,'fusion.simulink.internal.RandomNumberGenerator')
                error(message('fusion:simulink:RandomNumberGenerator:methodNotSupported', 'lastInitialSeed'));
            end

            seed = matlabshared.tracking.internal.SimulinkUtilities.seedManager(blkPath, true);
        end
    end
end