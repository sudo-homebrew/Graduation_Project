% This is an internal class and may be removed in future release.
% gaussEKFilter provides static methods for predicting and correcting the
% multiple mean and covariances from a Gaussian distribution using Extended
% Kalman filtering techniques.

% Copyright 2018 The MathWorks, Inc.

classdef gaussEKFilter
    %#codegen
    methods (Static)
        function [xk,P] = predict(x,P,Q,f,df,hasAPN,varargin)
            n = size(x,2);
            wZeros = zeros(size(Q,1),n,'like',x);
            if hasAPN
                xk = f(x,varargin{:});
            else
                xk = f(x,wZeros,varargin{:});
            end
            for i = 1:n
                Pi = P(:,:,i);
                if hasAPN
                    if isempty(df)
                        F = matlabshared.tracking.internal.numericJacobian(f, {x(:,i), varargin{:}});
                    else
                        F = df(x(:,i),varargin{:});
                    end
                    Pi = F*Pi*F' + Q;
                else
                    if isempty(df)
                        F = matlabshared.tracking.internal.numericJacobian(f, {x(:,i), wZeros(:,i), varargin{:}}, 1);
                        U = matlabshared.tracking.internal.numericJacobian(f, {x(:,i), wZeros(:,i), varargin{:}}, 2);
                    else
                        [F,U] = df(x(:,i),wZeros(:,i),varargin{:});
                    end
                    Pi = F*Pi*F' + U*Q*U';
                end
                P(:,:,i) = Pi;
            end
        end
                
        function zExp = getExpectedMeasurements(x, detectionCell, h, hasAMN, noiseSize)
            measParam = detectionCell{1}.MeasurementParameters;
            if hasAMN
                zExpi = callFcnWithMeasurementParams(h, measParam, x);
            else
                vZeros = zeros(noiseSize, size(x,2), 'like', x);
                zExpi = callFcnWithMeasurementParams(h, measParam, x, vZeros);
            end
            % Same measurement against each input measuerement
            zExp = repmat(zExpi, [numel(detectionCell) 1]);
        end
        
        function [H, U] = getMeasurementJacobians(x, detectionCell, h, dh, hasAMN, noiseSize, measSize)
            measParams = detectionCell{1}.MeasurementParameters;
            numDets = numel(detectionCell);
            if hasAMN
                % Additive measurement noise
                if ~isempty(dh)
                    Hi = callFcnWithMeasurementParams(dh, measParams, x);
                else
                    Hi = matlabshared.tracking.internal.numericJacobian(@(varargin)callFcnWithMeasurementParams(h,varargin{:}), {measParams, x}, 2);
                end
                U = eye(numDets*measSize,'like',x);
            else
                % Non-additive measurement noise
                vZeros = zeros(noiseSize, 1, 'like', x);
                if ~isempty(dh)
                    [Hi, Ui] = callFcnWithMeasurementParams(dh, measParams, x, vZeros);
                else
                    Hi = matlabshared.tracking.internal.numericJacobian(@(varargin)callFcnWithMeasurementParams(h,varargin{:}), {measParams, x, vZeros}, 2);
                    Ui = matlabshared.tracking.internal.numericJacobian(@(varargin)callFcnWithMeasurementParams(h,varargin{:}), {measParams, x, vZeros}, 3);
                end
                
                % Concatenate U's
                U = zeros(numDets*measSize,numDets*noiseSize,'like',x);
                for i = 1:numDets
                    rowIdx = ((i-1)*measSize + 1): i*measSize;
                    colIdx = ((i-1)*noiseSize + 1): i*noiseSize;
                    U(rowIdx,colIdx) = Ui;
                end
            end
            % Concatenate for calculation against each detection
            H = repmat(Hi,[numDets 1]);
        end
        
        function zExp = getExpectedExtentMeasurements(x, detectionCell, h, hasAMN, noiseSize)
            if hasAMN
                zExp = h(x, detectionCell);
            else
                vZeros = zeros(noiseSize,size(x,2),numel(detectionCell),'like',x);
                zExp = h(x, vZeros, detectionCell);
            end
        end
               
        function [H, U] = getExtentMeasurementJacobians(x, detectionCell, dh, hasAMN, noiseSize, measSize)
            numDets = numel(detectionCell);
            if hasAMN
                % Additive measurement noise
                if ~isempty(dh)
                    Hin = dh(x, detectionCell);
                end
                Uin = repmat(eye(measSize),[1 1 numDets]);
            else
                % Non-additive measurement noise
                vZeros = zeros(noiseSize, numel(detectionCell), 'like', x);
                if ~isempty(dh)
                    [Hin, Uin] = dh(x, vZeros, detectionCell);
                end
            end
            
            % Concatenate for calculation against each detection
            H = zeros(numDets*measSize, size(x,1), 'like', x);
            U = zeros(numDets*measSize, numDets*noiseSize, 'like', x);
            for i = 1:numDets
                index = ((i-1)*measSize + 1):(i*measSize);
                colIdx = ((i-1)*noiseSize + 1):(i*noiseSize);
                H(index,:) = Hin(:,:,i);
                U(index,colIdx) = Uin(:,:,i);
            end
        end
        
        function [x,P] = correct(x,P,r,S,H)
            n = size(x,2);
            for i = 1:n
                xi = x(:,i);
                Hi = H(:,:,i);
                Si = S(:,:,i);
                Pi = P(:,:,i);
                K = Pi*Hi'/Si;
                Pki = Pi - K*Hi*Pi;
                xki = xi + K*r(:,i);
                P(:,:,i) = Pki;
                x(:,i) = xki;
            end
        end
    end
end

function varargout = callFcnWithMeasurementParams(f, measParams, varargin)
    if iscell(measParams)
        [varargout{1:nargout}] = f(varargin{:},measParams{:});
    else
        [varargout{1:nargout}] = f(varargin{:},measParams);
    end
end