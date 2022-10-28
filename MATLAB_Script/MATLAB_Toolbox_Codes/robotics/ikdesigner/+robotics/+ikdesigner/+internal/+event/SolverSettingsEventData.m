classdef (ConstructOnLoad) SolverSettingsEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%SolverSettingsEventData Event data for sending and receiving solver settings data

%   Copyright 2021 The MathWorks, Inc.

    properties
        %SolverAlgorithm String indicating the solver algorithm
        SolverAlgorithm

        %SolverParameters Structure of solver parameters
        SolverParameters
    end

    methods
        function data = SolverSettingsEventData(solverAlgorithm, solverParameters)

            data.SolverAlgorithm = solverAlgorithm;
            data.SolverParameters = solverParameters;
        end
    end
end
