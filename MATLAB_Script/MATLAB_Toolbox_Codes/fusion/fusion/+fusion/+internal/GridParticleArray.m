classdef GridParticleArray < fusion.internal.MapInterface
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % This class represents an array of particles associated with a grid
    % map
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties
        Particles
        Weights
        CellIndex
    end
    
    properties
        NumParticles
        NumStateVariables
    end
    
    % For association with grid cell index
    properties
        ParticlePositionFcn
        ParticlePositionParameters
    end
    
    % Prediction properties
    properties
        StateTransitionFcn
        HasAdditiveProcessNoise
        ProcessNoise
        ProcessNoiseSamplingFcn
    end
    
    % For generating new particles
    properties
        ParticleSamplingFcn
        StateLimits
    end
    
    % Association with Grid
    properties (Access = {?fusion.internal.GridParticleArray, ?matlab.unittest.TestCase})
        StartIndex
        EndIndex
        CellXPositions
        CellYPositions
        CellWidth
    end
    
    properties (Access = protected)
        pIsPredictionEnsured = false;
        pIsCorrectionEnsured = false;
        pIsInitializationEnsured = false;
        pProdGridSize
    end
    
    properties
        ClassToUse = 'double';
        UseGPU = false;
    end
    
    methods
        function obj = GridParticleArray(varargin)
            % Set properties via name-value pairs
            matlabshared.fusionutils.internal.setProperties(obj,numel(varargin),varargin{:});
            
            if obj.UseGPU
                dummyVar = zeros(1,obj.ClassToUse,'gpuArray');
            else
                dummyVar = zeros(1,obj.ClassToUse);
            end
            
            p = zeros(obj.NumStateVariables,obj.NumParticles,'like',dummyVar);
            w = ones(obj.NumParticles,1,'like',dummyVar)/obj.NumParticles;
            ci = zeros(obj.NumParticles,1,'like',dummyVar);
            
            obj.Particles = p;
            obj.Weights = w;
            obj.CellIndex = ci;
        end
    end
    
    methods
        function set.Weights(obj, val)
            assert(~any(val(:) < 0));
            obj.Weights = val(:);
        end
    end
    
    methods (Access = protected)
        function f = defaultSamplingFcn(~)
            f = @gaussianSampler;
        end
    end
    
    %% Initialization
    methods
        function initialize(obj, measEvidence, varargin)
            % Assign same grid properties as the measurement map
            obj.GridSize = measEvidence.GridSize;
            obj.Resolution = measEvidence.Resolution;
            obj.GridOriginInLocal = measEvidence.GridOriginInLocal;
            obj.Length = measEvidence.Length;
            obj.Width = measEvidence.Width;
            
            % Assign product of grid size
            if ~coder.internal.is_defined(obj.pProdGridSize)
                obj.pProdGridSize = prod(obj.GridSize);
            end
            
            % Assign parameters controlling positional information from the
            % state in tracking coordinates
            obj.ParticlePositionParameters = measEvidence.PositionParameters;
            
            obj.CellXPositions = zeros(obj.GridSize,'like',obj.Particles);
            obj.CellYPositions = zeros(obj.GridSize,'like',obj.Particles);
            obj.CellWidth = zeros(1,'like',obj.Particles);
            [obj.CellXPositions(:), obj.CellYPositions(:), obj.CellWidth(:)] = calculateMapProperties(obj);
            
            % Initialization of particles and weights given occupied
            % evidence
            m_occ = getEvidences(measEvidence);
            [obj.Particles, obj.Weights, obj.CellIndex] = sampleUnassociated(obj, m_occ, obj.NumParticles);
            
            % Calculate cell index
            assignParticlesToGrid(obj);
        end
        
        function reinitialize(obj, measEvidence)
            % reinitialize(filter, measMap) reinitializes the filter using
            % the measurement evidence map.
            obj.ParticlePositionParameters = measEvidence.PositionParameters;
            
            m_occ = getEvidences(measEvidence);
            [obj.Particles, obj.Weights, obj.CellIndex] = sampleUnassociated(obj, m_occ, obj.NumParticles);
            
            % Calculate cell index
            assignParticlesToGrid(obj);
        end
    end
    
    methods (Access = {?fusion.internal.GridParticleArray,...
                       ?matlab.unittest.TestCase})
        function ensureInitializationIsDefined(obj)
            if ~obj.pIsInitializationEnsured
                coder.internal.assert(coder.internal.is_defined(obj.ParticlePositionFcn),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','ParticlePositionFcn','initialize');
                coder.internal.assert(coder.internal.is_defined(obj.ParticleSamplingFcn),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','ParticleSamplingFcn','initialize');  
                coder.internal.assert(coder.internal.is_defined(obj.StateLimits),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','StateLimits','initialize');  
                obj.pIsInitializationEnsured = true;
            end
        end
    end
    
    %% Prediction
    methods
        function predict(obj, dT, Ps)
            % predict(obj, dT, Ps) predicts the particle array dT time
            % ahead with a surival probability of Ps.
            predictParticles(obj, dT, Ps);
            assignParticlesToGrid(obj);
            truncateParticleWeights(obj, Ps);
        end
        
    end
    
    methods (Access = {?fusion.internal.GridParticleArray,...
            ?matlab.unittest.TestCase})
        function ensurePredictionIsDefined(obj)
            if ~obj.pIsPredictionEnsured
                ensureSamplingFcnIsDefined(obj);
                coder.internal.assert(obj.pIsInitializationEnsured,'fusion:internal:DynamicMapRFSFilter:MustCallBeforeUse','initialize','predict');
                coder.internal.assert(coder.internal.is_defined(obj.ProcessNoise),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','ProcessNoise','predict');
                coder.internal.assert(coder.internal.is_defined(obj.HasAdditiveProcessNoise),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','HasAdditiveProcessNoise','predict');
                coder.internal.assert(coder.internal.is_defined(obj.StateTransitionFcn),'fusion:internal:DynamicMapRFSFilter:MustDefineBeforeUse','StateTransitionFcn','predict');
                obj.pIsPredictionEnsured  = true;
            end
        end
        
        function ensureSamplingFcnIsDefined(obj)
            if ~coder.internal.is_defined(obj.ProcessNoiseSamplingFcn)
                obj.ProcessNoiseSamplingFcn = obj.defaultSamplingFcn;
            end
        end
        
        function predictParticles(obj, dT, Ps)
            % Generate noise samples
            v = obj.ProcessNoiseSamplingFcn(obj);
            
            % Predict particle states
            if obj.HasAdditiveProcessNoise
                obj.Particles = obj.StateTransitionFcn(obj.Particles, dT) + v;
            else
                obj.Particles = obj.StateTransitionFcn(obj.Particles, v, dT);
            end
            
            % Scale by Ps
            obj.Weights = Ps*obj.Weights;
        end
        
        function noise = gaussianSampler(obj)
            %GAUSSIANSAMPLER Generates a sample of noise from ProcessNoise
            % matrix by assuming Gaussian distribution.
            % The noise sample is used for predicting particles at the next
            % time step when process noise is additive or non-additive.
            %
            % The input to the function is the GridParticleArray object.
            %
            % The output of the function is a M-by-N matrix, where M is the
            % number of process noise terms and N is the number of
            % particles.
            %
            % Set the ProcessNoiseSamplingFcn property of the GridParticleArray to
            % a custom function following the input/output specification
            % to use a custom noise sampling function.
            % Example:
            % PF = fusion.internal.GridParticleArray;
            % noise = gaussianSampler(PF);
            
            processNoise = obj.ProcessNoise;
            sampler = matlabshared.tracking.internal.NormalDistribution(size(processNoise,1));
            sampler.Covariance = processNoise;
            sampler.Mean = zeros(1,size(processNoise,1));
            noise = sampler.sample(obj.NumParticles,'column');
        end
        
        function assignParticlesToGrid(obj)
            obj.CellIndex = calculateCellIndex(obj);
            
            [obj.CellIndex, idx] = sort(obj.CellIndex);
            
            % Sorted particles and weights
            obj.Particles = obj.Particles(:,idx);
            obj.Weights = obj.Weights(idx);
            
            N = obj.NumParticles;
            
            particleIdx = cast(1:N,'like',obj.Particles);

            sz = obj.pProdGridSize;
            
            particleCellIndex = obj.CellIndex;
            pInside = particleCellIndex(~isnan(particleCellIndex));
            particleIdxInside = particleIdx(~isnan(particleCellIndex));
            
            startIdx = accumarray(pInside(:),particleIdxInside,[sz 1],@min);
            endIdx = accumarray(pInside(:),particleIdxInside,[sz 1],@max);
            
            obj.StartIndex = reshape(startIdx, obj.GridSize);
            obj.EndIndex = reshape(endIdx, obj.GridSize);
        end
        
        function cellIndex = calculateCellIndex(obj)
            particles = obj.Particles;
            zParticles = obj.ParticlePositionFcn(particles, obj.ParticlePositionParameters);
            
            px = zParticles(1,:);
            py = zParticles(2,:);
            
            % Particles outside the limits
            xLimits = obj.XLocalLimits;
            yLimits = obj.YLocalLimits;
            outsideX = px < xLimits(1) | px > xLimits(2);
            outsideY = py < yLimits(1) | py > yLimits(2);
            
            outside = outsideX | outsideY;
            
            px(outside) = nan;
            py(outside) = nan;
            obj.Weights(outside) = 0;
            
            % Grid Indices
            gridIndices = local2gridImpl(obj,[px(:) py(:)]);
            
            % Cell Index
            cellIndex = obj.CellIndex;
            cellIndex(~outside) = sub2ind(obj.GridSize,gridIndices(~outside,1),gridIndices(~outside,2));
            cellIndex(outside) = nan;
        end
        
        function truncateParticleWeights(obj, Ps)
            weights = obj.Weights;
            cellIndex = obj.CellIndex;
            accumWeights = cumsum(weights);
            startIdx = obj.StartIndex;
            endIdx = obj.EndIndex;
            weightsPerCell = subtractaccum(accumWeights,startIdx,endIdx);
            cellToTruncate = weightsPerCell > Ps;
            
            validParticles = ~isnan(cellIndex);
            
            particleToTruncate = false(obj.NumParticles,1,'like',cellToTruncate);
            particleToTruncate(validParticles) = cellToTruncate(cellIndex(validParticles));
            
            currentCellSum = zeros(obj.NumParticles,1,'like',weights);
            currentCellSum(validParticles) = weightsPerCell(cellIndex(validParticles));
            weights(particleToTruncate) = weights(particleToTruncate)./currentCellSum(particleToTruncate)*Ps;
            obj.Weights = weights;
        end
    end
    
    %% Correction
    methods
        function [state, stateCov] = correct(obj, m_occ_pred, rho_b, rho_p, numBirthParticles, varargin)
            % No measurements are used, only being corrected by
            % evidence grids. By providing an association probability and a
            % likelihood of association with the measurement, the filter
            % can be enhanced to incorporate non-positional measurements
            % such as the range-rate or doppler from radar. 
            lhood = ones(obj.NumParticles,1,'like',obj.Particles);
            Pa = zeros(obj.GridSize,'like',obj.Particles);
            Pa_particle = zeros(obj.NumParticles,1,'like',obj.Particles);
            
            updatePersistentParticles(obj,lhood,Pa_particle,rho_p,m_occ_pred);
            
            % Get the state and state covariance estimate before
            % resampling. The state and state covariance defines the
            % persistent targets only. The born targets appear in the next
            % time-step.
            [state, stateCov] = getStateEstimate(obj);
            
            % Now we need to consider particle birth
            [pB, wB, ciB] = sampleNewParticles(obj, Pa, rho_b, numBirthParticles, varargin{:});
            
            % Total mass after resampling
            m = rho_p + rho_b;
            
            % Resample
            resample(obj, m, pB, wB, ciB);
        end
    end
    
    methods (Access = {?fusion.internal.GridParticleArray,...
            ?matlab.unittest.TestCase})
        function updatePersistentParticles(pf, g, pa_particle, rho_p, m_occ_pred)
            % Current weights
            weights = pf.Weights;
            
            % Associated particle weights
            weights_a = g.*weights;
            
            weight_array_accum = cumsum(weights_a);
            
            sumwc = subtractaccum(weight_array_accum, pf.StartIndex, pf.EndIndex);
            
            N = pf.NumParticles;
            
            mu_a = rho_p./(sumwc + eps(pf.ClassToUse)^2);
            
            mu_ua = rho_p./(m_occ_pred + eps(pf.ClassToUse)^2);
            
            % Normalizing constant for each particle
            mu_a_particle = zeros(N, 1, 'like', pf.Particles);
            mu_ua_particle = zeros(N, 1, 'like', pf.Particles);
            weightsU = zeros(N,1,'like', pf.Particles);
            
            particleCellIndex = pf.CellIndex;
            
            valid = ~isnan(particleCellIndex);
            
            mu_a_particle(valid) = mu_a(particleCellIndex(valid));
            mu_ua_particle(valid) = mu_ua(particleCellIndex(valid));
            
            weightsU(valid) = pa_particle(valid).*mu_a_particle(valid).*weights_a(valid) + (1 - pa_particle(valid)).*mu_ua_particle(valid).*weights(valid);
            pf.Weights = weightsU;
        end
        
        function resample(obj, m_occ, pB, wB, ciB)
            % Now resample new particles using persistent and new particles
            pP = obj.Particles;
            wP = obj.Weights;
            ciP = obj.CellIndex;
            
            % Combined list of new particles
            newParticles = [pP pB];
            newWeights = [wP;wB];
            newCellIndex = [ciP;ciB];
            
            valid = ~isnan(newWeights) & ~isnan(newCellIndex) & newWeights > 0;
            
            % No valid particles are available, just set everything to 0,
            % no need to resample.
            if ~any(valid)
                obj.Particles(:) = 0;
                obj.Weights(:) = 0;
                obj.CellIndex(:) = 0;
                return;
            end
            
            newWeights = newWeights(valid);
            newParticles = newParticles(:,valid);
            newCellIndex = newCellIndex(valid);

            [newWeights,idx] = sort(newWeights);
            newCellIndex = newCellIndex(idx);
            newParticles = newParticles(:,idx);

            joint_array_sum = cumsum(newWeights);
            sum_w = joint_array_sum(end);

            rand_array = sum_w*rand(obj.NumParticles,1);
            joint_array_sum = sort([0;joint_array_sum]);

            sampledIndices = discretize(rand_array, joint_array_sum, 'IncludedEdge','right');

            particles = newParticles(:,sampledIndices);
            cellIndex = newCellIndex(sampledIndices);

            sz = obj.pProdGridSize;

            numSampledParticlesPerCell = accumarray(cellIndex(:),1,[sz 1]);
            newWeightsPerCell = m_occ(:)./numSampledParticlesPerCell;
            weights = newWeightsPerCell(cellIndex);
            
            obj.Particles = particles;
            obj.Weights = weights;
            obj.CellIndex = cellIndex(:);
        end
        
        function ensureCorrectionIsDefined(obj)
            if ~obj.pIsCorrectionEnsured
                coder.internal.assert(obj.pIsInitializationEnsured,'fusion:internal:DynamicMapRFSFilter:MustCallBeforeUse','initialize','correct');
                obj.pIsCorrectionEnsured = true;
            end
        end
    end
    
    %% Sampling new particles
    methods (Access = {?fusion.internal.GridParticleArray,...
                       ?matlab.unittest.TestCase})
        function [p, w, ci] = sampleNewParticles(obj, Pa, born_mass, numParticles, varargin)
            % Normalize born_mass with numParticles
            sumMass = sum(born_mass(:));
            
            born_mass_normalized = numParticles/(sumMass + eps(obj.ClassToUse)^2)*born_mass;
            
            % Associated born mass
            born_mass_a = Pa.*born_mass;
            
            % Number of associated particles
            vb_a = floor(sum(born_mass_normalized(:).*Pa(:)));
                
            % Number of unassociated particles;
            vb_ua = numParticles - vb_a;
            
            % Unassociated born mass
            born_mass_ua = born_mass - born_mass_a;
            
            [p_a, w_a, ci_a] = sampleAssociated(obj, born_mass_a, vb_a, varargin{:});
            
            [p_ua, w_ua, ci_ua] = sampleUnassociated(obj, born_mass_ua, vb_ua);
            
            p = [p_a p_ua];
            w = [w_a;w_ua];
            ci = [ci_a;ci_ua];
        end
        
        function [p, w, ci] = sampleAssociated(obj, born_mass, numParticles, varargin)
            [x, y, w, ci] = sampleParticleLocalPositions(obj, born_mass, numParticles);
            pos = [x;y];
            % Kinematics of associated particles can be improved by
            % sampling from the available measurements. In this
            % implementation, only occupied and free evidences are use and
            % no measurements are available. Associated particles are
            % sampled from StateLimits.
            p = obj.ParticleSamplingFcn(pos, obj.ParticlePositionParameters, obj.StateLimits);
        end
        
        function [p, w, ci] = sampleUnassociated(obj, born_mass, numParticles)
            % Sample unassociated particles using the StateLimits.
            [x, y, w, ci] = sampleParticleLocalPositions(obj, born_mass, numParticles);
            pos = [x;y];
            p = obj.ParticleSamplingFcn(pos, obj.ParticlePositionParameters, obj.StateLimits);
        end
        
        function [x, y, w, ci] = sampleParticleLocalPositions(obj, born_mass, numParticles)
            w = zeros(numParticles, 1, 'like', obj.Weights);
            
            % Normalized born_mass
            sumMass = sum(born_mass(:));
            
            if sumMass > 0
                % Perform floating point error correction for born_mass to
                % avoid large errors due to floor. For example, consider
                % the following code:
                %
                % x = (0.99 - eps)*ones(8,1);
                % out = floor(x/sum(x)*1000);
                % out is 124 instead of 125.
                val = born_mass/sum(born_mass(:))*numParticles;
                floored = floor(val);
                ceiled = ceil(val);
                low = ceiled - val <= 2*eps(val);
                val = floored;
                val(low) = ceiled(low);
                born_mass_n = val;
                
                normSum = sum(born_mass_n(:));
                % Usually we won't be able to match the normSum perfectly
                % due to floor. Just add a few left particles to the cell
                % with most particles.
                
                if normSum < numParticles
                    dN = numParticles - normSum;
                    [~,idx] = max(born_mass_n(:));
                    born_mass_n(idx) = born_mass_n(idx) + dN;
                end
                
                particle_orders = cumsum(born_mass_n(:));
                
                % Adding 0.1 to particle orders makes the discretization
                % into bins more stable. Consider the case when startIdx is
                % [0;0;500;500] with 500 particles, there is a chance that
                % 500th particle may be assigned cell 3 due to numerical
                % errors. Making it [0.1;0.1;500.1;500.1] will make sure
                % that is not the case.
                startIdx = [0;particle_orders] + 0.1;
                startIdx(end) = numParticles + 0.1;
                
                % Create idx
                idx = zeros(1,numParticles,'like',obj.Particles);
                idx(:) = 1:1:numParticles;
                
                % Cell index of particles
                ciA = discretize(idx, startIdx, 'IncludedEdge','right');
                ci = cast(ciA(:),'like',obj.Particles);
                
                % Number of particles in the same cell as this particle
                nu = born_mass_n(ci);
                
                % sum of weights in this cell of this particle
                sumw = born_mass(ci);
                
                % Weights of particles are equal per cell
                w(nu > 0) = sumw(nu > 0)./nu(nu > 0);
                
                cellX = obj.CellXPositions;
                cellY = obj.CellYPositions;
                cellWidth = obj.CellWidth;
                
                xParticleCell = cellX(ci);
                yParticleCell = cellY(ci);
                
                % A random value between -0.5 and 0.5 cell width
                xyDiff = cellWidth*(rand(2,numParticles,'like',cellX) - 1/2);
                
                x = xParticleCell(:)' + xyDiff(1,:);
                y = yParticleCell(:)' + xyDiff(2,:);
            else
                x = zeros(1,numParticles,'like',obj.CellXPositions);
                y = zeros(1,numParticles','like',obj.CellXPositions);
                ci = zeros(numParticles, 1, 'like', obj.CellIndex);
            end
        end
    end
    
    methods (Access = {?fusion.internal.GridParticleArray,...
                       ?matlab.unittest.TestCase})
        function m_occ = getOccupancyMass(obj)
            m_occ = subtractaccum(cumsum(obj.Weights),obj.StartIndex,obj.EndIndex);
        end
        
        function [state, stateCov] = getStateEstimate(obj)
            m_occ = getOccupancyMass(obj);
            
            x = obj.Particles;
            w = obj.Weights;
            startIdx = obj.StartIndex;
            endIdx = obj.EndIndex;
            
            wx = bsxfun(@times,w',x);
            cwx = cumsum(wx,2);
            state = bsxfun(@rdivide,vecsubtractaccum(cwx',startIdx,endIdx),m_occ);
            
            if nargout > 1
                numParticlesPerCell = subtractaccum(cast(1:obj.NumParticles,'like',obj.Particles),startIdx,endIdx);
                notValid = numParticlesPerCell < 2;
                m = obj.NumStateVariables;
                wxi = repmat(wx,m,1);
                xj =  repelem(x,m,1);
                wxixj = bsxfun(@times,wxi,xj);
                cwxixj = cumsum(wxixj,2);
                mi = repmat(state,1,1,m);
                if coder.target('MATLAB')
                    mj = repelem(state,1,1,m); 
                else 
                    % state is a multi-dimensional matrix, which cannot be
                    % repelemed in codegen using repelem
                    mj = coder.nullcopy(zeros(obj.GridSize(1),obj.GridSize(2),m^2,obj.ClassToUse));
                    for i = 1:m
                        mj(:,:,(i-1)*m + (1:m)) = repmat(state(:,:,i),[1 1 m]);
                    end
                end
                mimj = bsxfun(@times,mi,mj);
                stateCovFlat = bsxfun(@rdivide,vecsubtractaccum(cwxixj',startIdx,endIdx),m_occ) - mimj;
                diagIdx = sub2ind([m m],1:m,1:m);
                stateCovFlat(:,:,diagIdx) = max(sqrt(eps(obj.ClassToUse)),stateCovFlat(:,:,diagIdx));
                stateCovFlat(repmat(notValid,[1 1 m^2])) = nan(obj.ClassToUse);
                stateCov = reshape(stateCovFlat,obj.GridSize(1),obj.GridSize(2),m,m);
            end
        end
    end
    
    methods (Access = protected)
        function map = createDynamicMap(obj)
            map = dynamicEvidentialGridMap(obj.Length,obj.Width,obj.Resolution,...
                'GridOriginInLocal',obj.GridOriginInLocal,...
                'NumStateVariables',obj.NumStateVariables,...
                'UseGPU',obj.UseGPU,...
                'DataType',obj.ClassToUse);
        end
    end
    
    methods
        function reset(obj)
            obj.Particles(:) = 0;
            obj.Weights(:) = 0;
            obj.CellIndex(:) = 0;
        end
    end
    
    methods (Static, Hidden)
        function propList = getPropertyList
            propList = {'Particles',...
                'Weights',...
                'CellIndex',...
                'ParticlePositionFcn',...
                'ParticlePositionParameters',...
                'StateTransitionFcn',...
                'HasAdditiveProcessNoise',...
                'ProcessNoise',...
                'ProcessNoiseSamplingFcn',...
                'ParticleSamplingFcn',...
                'StateLimits',...
                'Length',...
                'Width',...
                'GridSize',...
                'Resolution',...
                'GridOriginInLocal',...
                'StartIndex',...
                'EndIndex',...
                'CellXPositions',...
                'CellYPositions',...
                'CellWidth',...
                'pIsPredictionEnsured',...
                'pIsCorrectionEnsured',...
                'pIsInitializationEnsured',...
                'pProdGridSize'};
        end
    end

    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'Length','Width','Resolution','GridSize','UseGPU','ClassToUse','NumStateVariables','GridOriginInLocal'};
        end
    end
end

function out = subtractaccum(in, startIdx, endIdx)
out = zeros(size(startIdx),'like',in);
valid = startIdx > 1 & endIdx > 0;
out(valid) = in(endIdx(valid)) - in(startIdx(valid) - 1);
first = startIdx == 1;
out(first) = in(endIdx(first));
end

function out = vecsubtractaccum(in, startIdx, endIdx)
outTemp = zeros(numel(startIdx),size(in,2),'like',in);
valid = startIdx > 1 & endIdx > 0;
outTemp(valid,:) = in(endIdx(valid),:) - in(startIdx(valid) - 1,:);
first = startIdx == 1;
outTemp(first,:) = in(endIdx(first),:);
out = reshape(outTemp,[size(startIdx) size(in,2)]);
end
