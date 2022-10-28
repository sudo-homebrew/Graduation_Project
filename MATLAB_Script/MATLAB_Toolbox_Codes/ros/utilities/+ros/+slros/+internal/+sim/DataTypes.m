classdef DataTypes
%This class is for internal use only. It may be removed in the future.

%DataTypes Data type conversions needed by ROS Simulink blocks

%   Copyright 2015-2018 The MathWorks, Inc.

%#codegen

    properties (Constant)
        %SimulinkStringType - Data type used for string parameters
        %   Since this is not a fundamental Simulink data type, the actual
        %   data type text is encapsulated here.
        SimulinkStringType = 'uint8[] (string)'
    end

    methods (Static)
        function scalar = isSimulinkDataTypeScalar( slDataType )
        %isSimulinkDataTypeScalar Check if a given data type refers to a scalar type

            switch (slDataType)
              case {'int32', 'double', 'boolean'}
                scalar = true;
              otherwise
                scalar = false;
            end
        end

        function mlDataType = simulinkToMatlab( slDataType )
        %simulinkToMatlab Convert Simulink data type to MATLAB data type

            switch slDataType
              case {'double', 'single', 'uint16', 'uint32', ...
                    'int8', 'int16', 'int32'}
                mlDataType = slDataType;
              case 'boolean'
                mlDataType = 'logical';
              case ros.slros.internal.sim.DataTypes.SimulinkStringType
                % This is a string
                mlDataType = 'uint8';
              otherwise
                error(message('ros:slros:getparam:DataTypeSourceNotValid', slDataType));
            end

        end

        function slDataType = matlabToSimulinkTypeLabel( mlDataType )
        %matlabToSimulinkTypeLabel Convert MATLAB data type to Simulink data type label
        %   This function is used by the ParameterSelector dialog to
        %   display the data type for all parameters currently stored
        %   on the parameter server.

            switch mlDataType
              case {'double', 'single', 'uint16', 'uint32', ...
                    'int8', 'int16', 'int32'}
                slDataType = mlDataType;
              case 'uint8'
                slDataType = ros.slros.internal.sim.DataTypes.SimulinkStringType;
              case 'logical'
                slDataType = 'boolean';
              case 'char'
                slDataType = ros.slros.internal.sim.DataTypes.SimulinkStringType;
              case 'cell'
                % For now use ROS terminology for these data types
                slDataType = 'list';
              otherwise
                error(message('ros:slros:getparam:DataTypeSourceNotValid', mlDataType));
            end

        end


        function cppDataType = simulinkToCpp( slDataType )
        %simulinkToCpp Convert Simulink data type to C++ equivalent
        %   Use the standard Simulink type definitions here that are
        %   defined in rtwtypes.h. Please note that the actual data type
        %   might be different for various target platforms.

            switch slDataType
              case 'double'
                cppDataType = 'real64_T';
              case 'single'
                cppDataType = 'real32_T';
              case ros.slros.internal.sim.DataTypes.SimulinkStringType
                cppDataType = 'char_T';
              case {'uint8', 'uint16', 'uint32', 'int8', 'int16', 'int32', 'boolean'}
                cppDataType = [slDataType '_T'];
              otherwise
                error(message('ros:slros:getparam:DataTypeSourceNotValid', slDataType));
            end

        end

        function rosCppDataType = simulinkToROSCpp( slDataType, isArray )
        %simulinkToROSCpp Convert Simulink data type to ROS C++ equivalent
        %   Note that the returned C++ data type might be different
        %   from the C++ data type that Simulink is using and that is
        %   returned by simulinkToCpp.
        %   For example, the Simulink "boolean" data type maps to
        %   "unsigned char" in Simulink C++ code, but "bool" in ROS C++
        %   code.

        % Treat strings separately, since std::string is a special
        % array type.
            if strcmp(slDataType, ros.slros.internal.sim.DataTypes.SimulinkStringType)
                rosCppDataType = 'std::string';
                return;
            end

            % Handle all other data types
            % If they are arrays, they are treated as vectors in the C++
            % code.
            switch slDataType
              case 'double'
                rosCppDataType = 'double';
              case 'int32'
                rosCppDataType = 'int';
              case 'boolean'
                rosCppDataType = 'bool';
              otherwise
                error(message('ros:slros:getparam:DataTypeSourceNotValid', slDataType));
            end

            if isArray
                rosCppDataType = ['std::vector<' rosCppDataType '> '];
            end
        end
    end

end
