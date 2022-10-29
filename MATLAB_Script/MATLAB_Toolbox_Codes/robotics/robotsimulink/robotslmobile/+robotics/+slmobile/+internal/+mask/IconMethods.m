classdef IconMethods
    %ICONMETHODS Methods for updating the block mask icon (the face of the block)
	
	% Copyright 2019 The MathWorks, Inc.
    
    methods (Static)
        function [inportNames, outportNames] = updateUnicyclePortLabels(VehicleInputs)
            %UPDATEUNICYCLEPORTLABELS Update inport and outport names for unicycle kinematic model
            %   This method updates the inport and outport labels, which
            %   are passed to port_label commands in the block mask
            
            switch VehicleInputs
                case 'Wheel Speed & Heading Angular Velocity'
                    inportNames{1} = 'd\phi/dt';
                    inportNames{2} = '\omega';
                case 'Vehicle Speed & Heading Angular Velocity'
                    inportNames{1} = 'v'; %v
                    inportNames{2} = '\omega';
            end
            
            outportNames = {'state', 'stateDot'};
        end
        
        function [inportNames, outportNames] = updateBicyclePortLabels(VehicleInputs)
            %UPDATEBICYCLEPORTLABELS Update inport and outport names for bicycle kinematic model
            %   This method updates the inport and outport labels, which
            %   are passed to port_label commands in the block mask
            
            switch VehicleInputs
                case 'Vehicle Speed & Steering Angle'
                    inportNames{1} = 'v';
                    inportNames{2} = '\psi';
                case 'Vehicle Speed & Heading Angular Velocity'
                    inportNames{1} = 'v';
                    inportNames{2} = '\omega';
            end
            
            outportNames = {'state', 'stateDot'};
        end
        
        function [inportNames, outportNames] = updateDiffDrivePortLabels(VehicleInputs)
            %UPDATEDIFFDRIVEPORTLABELS Update inport and outport names for differential drive kinematic model
            %   This method updates the inport and outport labels, which
            %   are passed to port_label commands in the block mask
            
            switch VehicleInputs
                case 'Wheel Speeds'
                    inportNames{1} = 'd\phi_L/dt';
                    inportNames{2} = 'd\phi_R/dt';
                case 'Vehicle Speed & Heading Angular Velocity'
                    inportNames{1} = 'v';
                    inportNames{2} = '\omega';
            end
            
            outportNames = {'state', 'stateDot'};
        end
        
        
        function [inportNames, outportNames] = updateAckermannPortLabels()
            %UPDATEACKERMANNPORTLABELS Update inport and outport names for Ackermann kinematic model
            %   This method updates the inport and outport labels, which
            %   are passed to port_label commands in the block mask
            
            inportNames = {'v', 'd\psi/dt'};            
            outportNames = {'state', 'stateDot'};
        end
    end
end

