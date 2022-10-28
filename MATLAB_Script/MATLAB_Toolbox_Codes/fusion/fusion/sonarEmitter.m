classdef(Sealed, StrictDefaults) sonarEmitter < fusion.internal.remotesensors.ScanningEmitter & ...
        fusion.internal.remotesensors.mixin.DisplayUtils & ...
        radarfusion.internal.scenario.mixin.Emitter & ...
        scenario.internal.mixin.Perturbable

%sonarEmitter  Acoustic signals and interferences generator
%   emitter = sonarEmitter(emitterIndex) returns an acoustic frequency
%   emitter that simulates sonar emissions.
%     
%   emitter = sonarEmitter(emitterIndex, scanConfig) configures the sonar
%   emitter to use a predefined scan configuration, scanConfig. scanConfig
%   can be one of 'No scanning' | 'Raster' | 'Rotator' | 'Sector'.
%
%   emitter = sonarEmitter(..., 'Name', value) returns an sonarEmitter
%   object by specifying its properties as name-value pair arguments.
%   Unspecified properties have default values. See the list of properties
%   below.
%
%   Step method syntax:
%   
%   EMSONAR = step(EMITTER, PLAT, TIME) simulates a signal emitted from the
%   platform the emitter is mounted on, PLAT, at the simulation time, TIME.
%   PLAT is either a scalar struct of the current platform's pose or a
%   platform object as returned from the platform method on
%   trackingScenario or attached to the scenario's Platform property. TIME
%   is a scalar value in seconds. EMSONAR is a scalar struct representing
%   the sonar emission. The sonar emission updates at the rate defined by
%   the UpdateRate property.
%
%   A platform structure is a structure which can have the fields listed 
%   <a href="matlab:help fusion.internal.interfaces.DataStructures/platformStruct">here</a>.
%
%   EMSONAR is a sonarEmission object. Its properties are listed <a href="matlab:help sonarEmission">here</a>.
%
%   [EMSONAR, CONFIG] = step(EMITTER, PLAT, TIME) optionally returns the
%   current configuration of the emitter as an emitter configuration
%   struct. Its fields are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/emitterConfigStruct">here</a>.
%
%   sonarEmitter properties:
%     EmitterIndex              - Unique identifier of emitter system
%     UpdateRate                - Emitter update rate
%     MountingLocation          - Emitter's mounting location on platform
%     MountingAngles            - Emitter's mounting angles on platform
%     FieldOfView               - Angular field of view
%     ScanMode                  - Scan mode used by emitter
%     ElectronicScanLimits      - Electronic beam scan limits
%     ElectronicAngle           - Electronic beam scan angle (read-only)
%     LookAngle                 - Look angle of emitter (read-only)
%     HasElevation              - Enable elevation scanning
%     SourceLevel               - Sonar source level
%     CenterFrequency           - Center frequency of emitter
%     Bandwidth                 - Half power bandwidth of emitter
%     WaveformType              - Type of waveform emitted
%     ProcessingGain            - Processing gain from emitted waveform
%
%   sonarEmitter methods:
%     step            - Emit sonar emission
%     perturbations   - Define perturbations to the sonarEmitter
%     perturb         - Apply perturbations to the sonarEmitter
%     release         - Allow property value and input characteristics changes
%     clone           - Create sonarEmitter object with same property values
%     isLocked        - Locked status (logical)
%     reset           - Reset states of sonarEmitter object
%     <a href="matlab:help coverageConfig">coverageConfig</a>  - Report the sonarEmitter object scanning coverage configuration
%
%   % EXAMPLE: Model a sonar emitter.
%
%   % Create a platform to mount the emitter on.
%   plat = struct( ...
%       'PlatformID', 1, ...
%       'Position', [0 0 0]);
% 
%   % create the emitter
%   emitter = sonarEmitter(1, 'No scanning');
%
%   % Emit the waveform.
%   time = 0;
%   emTx = emitter(plat, time)
%
%   See also: sonarSensor, underwaterChannel, sonarEmission,
%   trackingScenario.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen
    
    properties(Nontunable)
        %SourceLevel Sonar source level
        %   Specify the sonar source level of the transmitter in decibels
        %   relative to the intensity of a sound wave having an rms
        %   pressure of 1 micropascal.  Units are in dB / 1 micropascal.
        %
        %   Default: 140
        SourceLevel = 140
        
        %CenterFrequency Center frequency of emitter
        %   Specify the center frequency (in Hz) of the operational band
        %   for the transmitter as a scalar.
        %
        %   Default: 20000
        CenterFrequency = 20000
        
        %Bandwidth Half power bandwidth of emitter
        %   Specify the bandwidth (in Hz) of the operational band for the
        %   transmitter as a scalar.
        %
        %   Default: 2000
        Bandwidth = 2000

        %ProcessingGain Processing gain from emitted waveform
        %   Processing gain achieved (in dB) when demodulating the emitted
        %   signal's waveform. Processing gain is achieved by emitting a
        %   signal over a bandwidth which is greater than the minimum
        %   bandwidth necessary to send the information contained in the
        %   signal.
        %
        %   Default: 0
        ProcessingGain = 0
        
        %WaveformType Type of waveform
        %   A user-defined nonnegative integer used to classify the type of
        %   waveform transmitted.
        %
        %   Default: 0
        WaveformType = 0
    end

    properties(Nontunable)
        %ScanMode  Scan mode used by sonar
        %   Specify the scan mode used by the sonar as one of 'No scanning'
        %   or 'Electronic'.  When set to 'No scanning', no scanning is 
        %   performed by the sonar. The sonar beam is along the antenna's
        %   boresight. When set to 'Electronic', the sonar scans electronically
        %   across the azimuth and elevation limits specified by the 
        %   ElectronicScanLimits property; the scan positions step
        %   by the sonar's field of view between dwells.
        %
        %   Default: 'Electronic'
        ScanMode = 'Electronic'
    end
    
    properties(Constant, Hidden)
        ScanModeSet = matlab.system.StringSet({'No scanning','Electronic'});
    end
    
    properties(Nontunable, Access = private)
        pUseQuat
    end
    
    properties(Access = private)
        pSignal
    end
    
    % -------------------
    % Setters and getters
    % -------------------
    methods
        function set.SourceLevel(obj,val)
            obj.checkScalarRealFinite(val, mfilename, 'SourceLevel');
            obj.SourceLevel = val;
        end
        
        function set.CenterFrequency(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'CenterFrequency');
            obj.CenterFrequency = val;
        end
        
        function set.Bandwidth(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'Bandwidth');
            obj.Bandwidth = val;
        end
        
        function set.ProcessingGain(obj,val)
            obj.checkScalarRealFinite(val, mfilename, 'ProcessingGain');
            obj.ProcessingGain = val;
        end
        
        function set.WaveformType(obj,val)
            obj.checkScalarNonnegativeIndex(val, mfilename, 'WaveformType');
            obj.WaveformType = val;
        end
    end
    
    % -----------
    % Constructor
    % -----------
    methods
        function obj = sonarEmitter(varargin)
            % Support name-value pair arguments when constructing object            
            obj@fusion.internal.remotesensors.ScanningEmitter(varargin{:});
        end
    end

    % --------------
    % Implementation
    % --------------
    methods(Hidden)
        function flag = isSignalGenerator(~)
            % Returns true if device returns signals
            flag = true;
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj,platIn,~)
            setupImpl@fusion.internal.remotesensors.ScanningEmitter(obj);
            
            if isstruct(platIn)
                plat = fusion.internal.interfaces.DataStructures.assembleStructs(platIn);
            else
                plat = platIn;
            end
            quat = plat.Orientation;
            obj.pUseQuat = isa(quat, 'quaternion');
            obj.pSignal = defaultOutput(obj);
        end
        
        function [signalOut, config] = stepImpl(obj,platIn,time)
            
            step(obj.pScanner,time);
            
            isValidTime = isValidUpdateTime(obj, time);
            if isValidTime
                if isstruct(platIn)
                    plat = fusion.internal.interfaces.DataStructures.assembleStructs(platIn);
                else
                    plat = platIn;
                end
                
                signal = obj.pSignal;
                
                signal.EmitterIndex = obj.EmitterIndex;
                signal.PlatformID = plat.PlatformID;

                % Convert from body to scenario frame
                quat = plat.Orientation;
                if obj.pUseQuat
                    rotPlat = fusion.internal.frames.quat2rotmat(quat);
                else
                    rotPlat = quat;
                end
                originMount = obj.MountingLocation(:);
                originSig = plat.Position(:)+rotPlat'*originMount;
                
                % Convert look angle (az,el) to axes in mount frame
                rotLook = getLookOrientation(obj);
                
                rotSig = rotPlat*rotLook;
                if obj.pUseQuat
                    orientSig = fusion.internal.frames.rotmat2quat(rotSig);
                else
                    orientSig = rotSig;
                end
                
                signal.OriginPosition(:) = originSig;
                signal.OriginVelocity(:) = plat.Velocity;
                signal.Orientation(:) = orientSig;
                signal.FieldOfView(:) = obj.FieldOfView;
                
                signal.SourceLevel = obj.SourceLevel;
                signal.CenterFrequency = obj.CenterFrequency;
                signal.Bandwidth = obj.Bandwidth;
                signal.ProcessingGain = obj.ProcessingGain;
                signal.WaveformType = obj.WaveformType;
                obj.pSignal = signal;
                
                % Latch current step time
                obj.pHasFirstUpdate = true;
                obj.pTimeLastUpdate = time;
            end
            signalOut = obj.pSignal;
            
            scanDone = isValidTime && isScanDone(obj.pScanner);
            transforms = configTransforms(obj,[],obj.pHasElevation,false,false);
            config = systemConfig(obj,isValidTime,scanDone,transforms);
        end
        
        function resetImpl(obj)
            resetImpl@fusion.internal.remotesensors.ScanningEmitter(obj);
        end

        function [out, argsToBus] = defaultOutput(obj)
            argsToBus = {};
            if obj.pUseQuat
                orient = quaternion.ones;
            else
                orient = eye(3);
            end
            out = sonarEmission('Orientation',orient);
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = true;
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end
    end
    
    % -------------
    % Save and load
    % -------------
    methods(Access=protected)
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            if wasLocked
                obj.pUseQuat = s.pUseQuat;
                obj.pSignal = s.pSignal;
            end
            
            % Set perturbation related properties
            loadPerts(obj,s);
            
            % Set public properties and states
            loadObjectImpl@fusion.internal.remotesensors.ScanningEmitter(obj,s,wasLocked);
       end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@fusion.internal.remotesensors.ScanningEmitter(obj);

            % Set private and protected properties
            if isLocked(obj)
                s.pUseQuat = obj.pUseQuat;
                s.pSignal = obj.pSignal;
            end
            
            % Set perturbation related properties
            s = savePerts(obj, s);
        end
    end
    
    % ----------------
    % Display settings
    % ----------------
    
    % General
    methods(Access = protected)
        function flag = isInactivePropertyImpl(obj,prop)
            if any(strcmp(prop,{'MechanicalAngle','LookAngle'}))
                flag = true;
            else
                flag = isInactivePropertyImpl@fusion.internal.remotesensors.ScanningEmitter(obj,prop);
            end
        end
    end
    
    % MATLAB display
    methods(Access=protected)
        function groups = getPropertyGroups(obj)
            groups = sonarEmitter.getPropertyGroupsImpl();
            groups = obj.convertSystemToMixinGroup(groups);
        end
    end
    
    % Simulink dialog
    methods(Access = protected, Static)
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            scanningGroups = getPropertyGroupsImpl@fusion.internal.remotesensors.ScanningEmitter;
            mainSection = scanningGroups(1);
            mntSection = scanningGroups(2);
            limitSection = scanningGroups(3);
            hasSection = scanningGroups(4);
            
            pList = {'SourceLevel','CenterFrequency','Bandwidth','WaveformType','ProcessingGain'};
            emitSection = matlab.system.display.Section('PropertyList',pList);
            
            limitSection.PropertyList = [limitSection.PropertyList,hasSection.PropertyList];
            
            groups = [mainSection, mntSection, limitSection, emitSection];
        end
    end
    
    % -------------------
    % Input validation
    % -------------------
    methods(Access = protected)
        function validateInputsImpl(obj, plat, time)
            % Validate inputs to the step method at initialization (or
            % whenever an input size changes)
            validateInputsImpl@fusion.internal.remotesensors.ScanningEmitter(obj, time);

            validateattributes(plat,{'struct','fusion.scenario.Platform'},{'scalar'},mfilename,'PLAT');
            if isstruct(plat)
                type = fusion.internal.interfaces.DataStructures.structType(plat);
                cond = ~(strcmpi(type, 'platformStruct') || strcmpi(type, 'platformPoseStruct')) || ~isfield(plat, 'PlatformID');
                if cond
                    if coder.target('MATLAB')
                        hereString = getString(message('shared_radarfusion:RemoteSensors:here'));
                        hereLink = ['<a href="matlab:help(''fusion.internal.interfaces.DataStructures/platformStruct'')">' hereString '</a>'];
                        errMsg = getString(message('shared_radarfusion:RemoteSensors:expectStructType','platform',hereLink,'PLAT'));
                        error('shared_radarfusion:RemoteSensors:expectStructType',errMsg);
                    else
                        coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:expectStructTypeCG','platform','PLAT');
                    end
                end
                
                obj.checkNumericFields(plat, mfilename, 'PLAT');
            end
        end
    end
    
    % -------------
    % Perturbations
    % -------------
    methods(Access = protected)
        function perts = defaultPerturbations(~)
            perturbableProps = {"SourceLevel", "CenterFrequency",...
                "Bandwidth", "WaveformType", "ProcessingGain"}; %#ok<CLARRSTR>
            perts = struct(...
                'Property', perturbableProps, ...
                'Type', "None", ...
                'Value', {{NaN, NaN}}...
                );
        end
        
        function intProps = integerProperties(~)
            %integerProperties Get the list of integer-valued properties
            intProps = "WaveformType";
        end
    end
    
    methods(Access = protected)
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            validatePropertiesImpl@fusion.internal.remotesensors.ScanningEmitter(obj);
        end
    end
    
    % --------
    % Simulink
    % --------
    methods(Static, Hidden)    
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end
