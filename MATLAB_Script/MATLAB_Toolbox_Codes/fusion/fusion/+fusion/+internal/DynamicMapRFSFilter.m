classdef DynamicMapRFSFilter < fusion.internal.GridParticleArray
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % This class implements the dynamic map estimator using the RFS method
    % proposed in [1].
    
    
    % filter = fusion.internal.DynamicMapRFSFilter(Name,value) creates the
    % filter. See the properties of the filter to get N/V pairs
    %
    % You can initialize a filter using initialize method
    % initialize(filter, measurementMap) where measurementMap is defined as
    % an object of fusion.internal.MeasurementEvidenceMap.
    %
    % If you are tracking in global frame, be sure to provide the
    % PositionParameters on the measurementMap.
    %
    % dynamicMap = predict(filter, dT) allows prediction of the dynamic
    % map.
    %
    % dynamicMap = correct(filter, measMap) allows correction of the
    % dynamic map.
    %
    % where dynamicMap is an object of type dynamicEvidentialGridMap
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties
        DeathRate = 1e-3
        FreeSpaceDiscountFactor = 0.8;
    end
    
    % New born particles per step
    properties
        NumBirthParticles = 1e4
        BirthProbability = 1e-2
    end
    
    % Dependent on the dynamic map
    properties (Access = {?fusion.internal.DynamicMapRFSFilter,?matlab.unittest.TestCase}, Dependent)
        State
        StateCovariance
        OccupiedEvidence
        FreeEvidence
    end
    
    properties (Access = {?fusion.internal.DynamicMapRFSFilter,?trackerGridRFS,?matlab.unittest.TestCase})
        DynamicMap
    end
    
    methods
        function val = get.State(obj)
            val = getStateEstimate(obj.DynamicMap);
        end
        
        function val = get.StateCovariance(obj)
            val = getStateCovarianceEstimate(obj.DynamicMap);
        end
        
        function val = get.OccupiedEvidence(obj)
            val = getOccupiedEvidence(obj.DynamicMap);
        end
        
        function set.OccupiedEvidence(obj,val)
            setOccupiedEvidence(obj.DynamicMap, val);
        end
        
        function val = get.FreeEvidence(obj)
            val = getFreeEvidence(obj.DynamicMap);
        end
        
        function set.FreeEvidence(obj,val)
            setFreeEvidence(obj.DynamicMap,val);
        end
    end
    
    methods
        function obj = DynamicMapRFSFilter(varargin)
            % Allocate memory for the filter
            obj@fusion.internal.GridParticleArray(varargin{:});
            
            matlabshared.fusionutils.internal.setProperties(obj,numel(varargin),varargin{:});
        end

        function initialize(obj, measurementEvidence)
            ensureInitializationIsDefined(obj);
            
            initialize@fusion.internal.GridParticleArray(obj, measurementEvidence);
            
            % Allocate memory for the dynamic map
            obj.DynamicMap = createDynamicMap(obj);
            
            [m_occ, m_free] = getEvidences(measurementEvidence);
            obj.OccupiedEvidence = m_occ;
            obj.FreeEvidence = m_free;
        end
        
        function reinitialize(obj, measurementEvidence)
            % reinitialize(obj, measMap) reinitializes the map without
            % memory allocation. It is required to allow a filter to reset
            % and then start back
            reinitialize@fusion.internal.GridParticleArray(obj, measurementEvidence);
            [m_occ, m_free] = getEvidences(measurementEvidence);
            obj.OccupiedEvidence = m_occ;
            obj.FreeEvidence = m_free;
        end
        
        function map = predict(obj, dT)
            ensurePredictionIsDefined(obj);
            
            % Survival probability
            Ps = (1 - obj.DeathRate)^dT;
            
            % Predict particle and grid arrays
            predict@fusion.internal.GridParticleArray(obj, dT, Ps);
            
            % Predict evidences Occupied evidence is simply the sum of
            % particle weights per cell
            obj.OccupiedEvidence = getOccupancyMass(obj);
            
            % Free evidence is modeled by a decay factor
            obj.FreeEvidence = min(obj.FreeSpaceDiscountFactor^dT*obj.FreeEvidence, 1 - obj.OccupiedEvidence);
            
            % Output the map
            map = obj.DynamicMap;
        end
        
        function map = correct(obj, measurementEvidence, varargin)
            ensureCorrectionIsDefined(obj);
            
            % Get input evidences
            [m_occ, m_free] = getEvidences(measurementEvidence);
            
            % Predicted occ and free evidences
            m_occ_pred = obj.OccupiedEvidence;
            m_free_pred = obj.FreeEvidence;
            
            % Update belief masses and distribute occupied evidence into
            % persistent and new born targets. b -> born, p -> persistent
            [m_occ_up, m_free_up, rho_b, rho_p] = updateCellOccupancy(m_occ_pred, m_free_pred, m_occ, m_free, obj.BirthProbability);
            
            % Correct the particle filter.
            [state, stateCov] = correct@fusion.internal.GridParticleArray(obj, m_occ_pred, rho_b, rho_p, obj.NumBirthParticles, varargin{:});
            
            % Update state of the filter
            obj.OccupiedEvidence = m_occ_up;
            obj.FreeEvidence = min(m_free_up,1 - m_occ_up);
            
            setMapStateAndCovariance(obj.DynamicMap, state, stateCov);
            
            % Output the map
            map = obj.DynamicMap;
        end
    end
    
    methods (Access = {?fusion.internal.GridParticleArray,?trackerGridRFS})
        function st = captureState(obj)
            % Particles states, weights and appropriate cell index is 
            % enough to calculate m_occ, state, state covariance
            st.Particles = obj.Particles;
            st.Weights = obj.Weights;
            st.FreeEvidence = obj.FreeEvidence;
        end
        
        function syncWithState(obj, st)
            % Set particles, weights.
            obj.Particles = st.Particles;
            obj.Weights = st.Weights;
            
            % Set evidences obtained from particles and weights.
            % Note setting the State and StateCovariance is not necessary
            % as they are just for output and are obtained from particles
            % at each step
            obj.FreeEvidence = st.FreeEvidence;
        end
        
        function map = predictForOutput(obj, dT, withStateAndCovariance)
            % Predict the filter regularly using predict method
            if dT > 0
                predict(obj, dT);
            else
                % Assign original particles to grid
                assignParticlesToGrid(obj);
                obj.OccupiedEvidence = getOccupancyMass(obj);
            end
            
            map = obj.DynamicMap;
            
            if withStateAndCovariance
                % Obtain state estimate and update the map output
                [state, stateCov] = getStateEstimate(obj);            
                setMapStateAndCovariance(map, state, stateCov);
            end
        end
    end
    
    %% Save/load/clone
    methods
        function s = saveobj(obj)
           propList = fusion.internal.DynamicMapRFSFilter.getPropertyList;
           s.NumParticles = obj.NumParticles;
           s.NumStateVariables = obj.NumStateVariables;
           s.UseGPU = obj.UseGPU;
           s.ClassToUse = obj.ClassToUse;
           
           for i = 1:numel(propList)
               if coder.internal.is_defined(obj.(propList{i}))
                   s.(propList{i}) = obj.(propList{i});
               end
           end
           if coder.internal.is_defined(obj.DynamicMap)
               s.DynamicMap = obj.DynamicMap;
           end
        end
        
        function newObj = clone(obj)
            newObj = fusion.internal.DynamicMapRFSFilter('NumParticles',obj.NumParticles,...
                'NumStateVariables',obj.NumStateVariables,...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.ClassToUse);
            
            propList = fusion.internal.DynamicMapRFSFilter.getPropertyList;
            
            for i = 1:numel(propList)
                if coder.internal.is_defined(obj.(propList{i}))
                    newObj.(propList{i}) = obj.(propList{i});
                end
            end
            
            % Deep copy of handles
            if coder.internal.is_defined(obj.DynamicMap)
                newObj.DynamicMap = clone(obj.DynamicMap);
            end
        end
    end
    
    methods
        function reset(obj)
            if coder.internal.is_defined(obj.DynamicMap)
                nullify(obj.DynamicMap);
            end
            reset@fusion.internal.GridParticleArray(obj);
        end
    end
    
    methods (Static)
        function obj = loadobj(s)
            obj = fusion.internal.DynamicMapRFSFilter('NumParticles',s.NumParticles,...
                'NumStateVariables',s.NumStateVariables,...
                'UseGPU',s.UseGPU,...
                'ClassToUse',s.ClassToUse);
            
            s = rmfield(s,'NumParticles');
            s = rmfield(s,'NumStateVariables');
            s = rmfield(s,'UseGPU');
            s = rmfield(s,'ClassToUse');
            
            propList = fieldnames(s);
            
            for i = 1:numel(propList)
                obj.(propList{i}) = s.(propList{i});
            end
            
            % Shallow copy of handle
            if isfield(s,'DynamicMap')
                obj.DynamicMap = s.DynamicMap;
            end
        end
    end
    
    methods (Static, Hidden)
        function propList = getPropertyList
            propList1 = fusion.internal.GridParticleArray.getPropertyList;
            propList2 = {'DeathRate',...
                'FreeSpaceDiscountFactor',...
                'NumBirthParticles',...
                'BirthProbability'};
            propList = {propList1{:} propList2{:}};
        end
    end
end

function [m_occ_up, m_free_up, rho_b, rho_p] = updateCellOccupancy(m_occ_pred, m_free_pred, m_occ, m_free, pb)
[m_occ_up, m_free_up] = fusion.internal.DempsterShaferOccupancyUpdate(m_occ_pred, m_free_pred, m_occ, m_free);
rho_b = m_occ_up.*(1 - m_occ_pred).*pb./(m_occ_pred + pb.*(1 - m_occ_pred));
rho_p = m_occ_up - rho_b;
rho_p = max(0, rho_p);
rho_b = max(0, rho_b);
m_occ_up = rho_b + rho_p;
m_free_up = max(0, m_free_up);
end