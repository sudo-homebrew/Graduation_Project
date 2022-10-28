classdef StateMachineController < handle
%This class is for internal use only. It may be removed in the future.

%StateMachineController This class encapsulates the Controller portion
%   of the Model/View/Controller design pattern used in the SLAM app for
%   handling app state

% Copyright 2018-2020 The MathWorks, Inc.

    properties
        AppViewOrg

        StateMachineModel
    end

    methods
        function obj = StateMachineController( stateMachineModel,  appViewOrg )
        %StateMachineController Constructor

            assert(isa(stateMachineModel, 'nav.slamapp.internal.StateMachineModel'));

            assert(isa(appViewOrg, 'nav.slamapp.internal.AppViewOrganizer'));

            obj.AppViewOrg = appViewOrg;
            obj.StateMachineModel = stateMachineModel;

            obj.addViewListeners();
            obj.addModelListeners();

        end

    end


    % listen to View events
    methods
        function addViewListeners(~)
        %addViewListeners


        end
    end


    % listen to Model events
    methods
        function addModelListeners(obj)
        %addModelListeners
            addlistener(obj.StateMachineModel, 'State', 'PostSet', @(source, event) obj.AppViewOrg.refreshAppViewAccordingToState(event.AffectedObject.State) );
        end
    end


    methods


    end
end
