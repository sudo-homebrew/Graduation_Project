classdef StateMachineModel < handle & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %STATEMACHINEMODEL Model to keep track of the SLAM app state

    % Copyright 2018-2022 The MathWorks, Inc.

    properties (SetObservable)
        State
    end

    properties
        PrevState
    end

    methods
        function obj = StateMachineModel()
        %STATEMACHINEMODEL Constructor
            import nav.slamapp.internal.*
            obj.State = States.Init;
            obj.PrevState = States.Init;
        end


        function backToInit(obj)
        %backToInit
            import nav.slamapp.internal.*
            assert(obj.State == States.LoadingBag || obj.State == States.LoadingFromWS);

            obj.PrevState = obj.State;
            obj.State = States.Init;

        end

        function toLoadingBag(obj)
        %toLoadingBag
            import nav.slamapp.internal.*
            assert(obj.State == States.Init);

            obj.PrevState = obj.State;
            obj.State = States.LoadingBag;

        end

        function toLoadingFromWorkspace(obj)
        %toLoadingFromWorkspace
            import nav.slamapp.internal.*
            assert(obj.State == States.Init)

            obj.PrevState = obj.State;
            obj.State = States.LoadingFromWS;
        end

        function toSensorDataReady(obj)
        %toSensorDataReady
            import nav.slamapp.internal.*
            assert(obj.State == States.LoadingBag || obj.State == States.LoadingFromWS);

            obj.PrevState = obj.State;
            obj.State = States.SensorDataReady;
        end

        function toMapping(obj)
        %toMapping
            import nav.slamapp.internal.*
            assert(obj.State == States.SensorDataReady || obj.State == States.MappingPaused);

            obj.PrevState = obj.State;
            obj.State = States.Mapping;
        end

        function toMappingPaused(obj)
        %toMappingPaused
            import nav.slamapp.internal.*
            assert(obj.State == States.Mapping || obj.State == States.ModifyingIncremental || obj.State == States.ModifyingLoopClosure || obj.State == States.MappingPaused);

            % slam app state machine changes it's state momentarily to
            % generate state change event. After catching the event app
            % starts to update it's UI which is slow when compared to state
            % machine update. Applying a guard here to only update it's
            % previous state when it's not in paused state already.
            if obj.State ~= States.MappingPaused
                obj.PrevState = obj.State;
            end
            obj.State = States.MappingPaused;
        end

        function toModifyingIncremental(obj)
        %toModifyingIncremental
            import nav.slamapp.internal.*
            assert(obj.State == States.MappingPaused || obj.State == States.Mapped);

            obj.PrevState = obj.State;
            obj.State = States.ModifyingIncremental;
        end

        function toModifyingLoopClosure(obj)
        %toModifyingLoopClosure
            import nav.slamapp.internal.*
            assert(obj.State == States.MappingPaused || obj.State == States.Mapped);

            obj.PrevState = obj.State;
            obj.State = States.ModifyingLoopClosure;
        end

        function toMapped(obj)
        %toMapped
            import nav.slamapp.internal.*
            assert(obj.State == States.Mapping || obj.State == States.MappingPaused);
            obj.State = States.Mapped;
        end

        function toPreviousState(obj)
        %toPreviousState
            import nav.slamapp.internal.*

            tmp = obj.PrevState;
            obj.PrevState = obj.State;
            obj.State = tmp;
        end

    end

    methods
        function [infoStruct, extraInfoStruct] = saveProperties(obj)
        %saveProperties

        % basic properties
            infoStruct = saveProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj);

            % observable properties
            extraInfoStruct.State = obj.State;
        end

        function loadProperties(obj, infoStruct1, infoStruct2)
        %loadProperties
            loadProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj, infoStruct1);

            % set the state
            obj.State = infoStruct2.State;
        end
    end
end
