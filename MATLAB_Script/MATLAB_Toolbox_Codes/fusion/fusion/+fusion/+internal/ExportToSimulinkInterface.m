classdef (Abstract,Hidden) ExportToSimulinkInterface < handle
    %   Provides a common interface for exporting a MATLAB object to
    %   Simulink.
    %
    %   This class is for internal use only and may be removed in a
    %   future release.

    %   Copyright 2021 The MathWorks, Inc.
    
    %#codegen
    methods
        function blkHandle = exportToSimulink(obj,varargin)
            % EXPORTTOSIMULINK Export the MATLAB object to a Simulink
            % model.
            %
            % EXPORTTOSIMULINK(OBJ) exports the MATLAB object, OBJ, as a
            % Simulink block in a new model with default name.
            %
            % EXPORTTOSIMULINK(..., 'Model', MODEL), allows you to export
            % the  MATLAB object to an existing Simulink model MODEL. MODEL
            % can be the name or handle of the Simulink model. If a
            % Simulink model with name MODEL does not exist, a new model is
            % created with name MODEL.
            %
            % EXPORTTOSIMULINK(..., 'BlockName', Name), allows you to
            % specify a name, NAME for the Simulink block.
            %
            % EXPORTTOSIMULINK(..., 'Position', POS), allows you to specify
            % block position, POS, in the model. POS must be a vector of
            % coordinates, in pixels: [left top right bottom]
            %
            % EXPORTTOSIMULINK(...,'OpenModel', TF), allows you to specify
            % a flag, TF, to indicate if the model should be opened after
            % exporting the OBJ to Simulink or not. The default value
            % is set to true which means the model is always opened.
            %
            % H = EXPORTTOSIMULINK(OBJ, ...) exports the  MATLAB object,
            % OBJ, as a Simulink block and returns the handle to the
            % block.
                        
            %Add block to the model and set mask parameters.           
            blkHandle = fusion.simulink.internal.addBlockAndSetParameters(obj,varargin{:});            
        end
    end
end