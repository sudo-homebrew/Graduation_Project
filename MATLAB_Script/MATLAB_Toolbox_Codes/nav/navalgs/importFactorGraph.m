function [G, nodeID, measurement, information, nodeIDPair, nodeEstimate] = importFactorGraph(fileName)
%IMPORTFACTORGRAPH Create a factorGraph object from g2o format log file
%   G = IMPORTFACTORGRAPH(FILENAME) returns a factorGraph object
%   by parsing the g2o format log file. The g2o log file can either only 
%   contain 'EDGE_SE2' and 'VERTEX_SE2' tokens, or only contain
%   'EDGE_SE3:QUAT' and 'VERTEX_SE3:QUAT' tokens. 
%
%   See also factorGraph

%   Copyright 2021 The MathWorks, Inc.

tmp = which(fileName);
if isempty(tmp)
    fileID = fopen(fileName);
else
    fileID = fopen(tmp);
end
coder.internal.errorIf(fileID == -1, 'nav:navalgs:factorgraph:CannotOpenFile');

% Prescan the log file to get some numbers
numEdges = 0;
numNodes = 0;
numLines = 0;
dim = -1; % unset
nextLine = fgetl(fileID);
while ischar(nextLine)
    numLines = numLines + 1;
    token = strtok(nextLine);
    
    if ~strcmp(token, 'EDGE_SE2') && ~strcmp(token, 'VERTEX_SE2') &&...
          ~strcmp(token, 'EDGE_SE3:QUAT') && ~strcmp(token, 'VERTEX_SE3:QUAT')
        fclose(fileID);
        coder.internal.error('nav:navalgs:factorgraph:UnexpectedTokenFound', numLines);
    end

    if dim == -1
        if strcmp(token, 'EDGE_SE2') || strcmp(token, 'VERTEX_SE2')
            dim = 2;
        elseif strcmp(token, 'EDGE_SE3:QUAT') || strcmp(token, 'VERTEX_SE3:QUAT')
            dim = 3;
        end
    end

    if (dim == 2 && (strcmp(token, 'EDGE_SE3:QUAT') || strcmp(token, 'VERTEX_SE3:QUAT'))) || ...
       (dim == 3 && (strcmp(token, 'EDGE_SE2') || strcmp(token, 'VERTEX_SE2')))
        fclose(fileID);
        coder.internal.error('nav:navalgs:factorgraph:MismatchedDimension', numLines);
    end

    if strcmp(token, 'EDGE_SE2') || strcmp(token, 'EDGE_SE3:QUAT')
        numEdges = numEdges + 1;
    elseif strcmp(token, 'VERTEX_SE2') || strcmp(token, 'VERTEX_SE3:QUAT' )
        numNodes = numNodes + 1;
    end
    nextLine = fgetl(fileID);
end

fclose(fileID);

% Extract the data

if dim == 2
    nodeIDPair = zeros(numEdges, 2);
    measurement = zeros(numEdges, 3);
    information = zeros(numEdges, 6);
    nodeEstimate = zeros(numNodes, 3);
    nodeID = zeros(numNodes, 1);
else
    nodeIDPair = zeros(numEdges, 2);
    measurement = zeros(numEdges, 7);
    information = zeros(numEdges, 21);
    nodeEstimate = zeros(numNodes, 7);
    nodeID = zeros(numNodes, 1);
end

fileID = fopen(fileName);

nextLine = fgetl(fileID);

edgeCnt = 0;
nodeCnt = 0;
lineCnt = 0;
while ischar(nextLine)
    lineCnt = lineCnt + 1;
    [token, remainingLine] = strtok(nextLine);

    if strcmp(token, 'EDGE_SE2')
        edgeCnt = edgeCnt + 1;
        % EDGE_SE2 eid1 eid2 dx dy dtheta I_11 I_12 I_13 I_22 I_23 I_33
        lineData = sscanf(remainingLine, '%f');
        lineData = lineData(:)';
        if numel(lineData) == 11 % expects 11 numeric values on this dataline
            nodeIDPair(edgeCnt,:) = [round(lineData(1)), round(lineData(2))];
            measurement(edgeCnt,:) = lineData(3:5);
            information(edgeCnt,:) = lineData(6:11);
        else
            fclose(fileID);
            coder.internal.error('nav:navalgs:factorgraph:InvalidDataLine', lineCnt);
        end

    elseif strcmp(token, 'VERTEX_SE2')
        nodeCnt = nodeCnt + 1;
        % VERTEX_SE2 vid x y theta
        lineData = sscanf(remainingLine, '%f');
        lineData = lineData(:)';
        if numel(lineData) == 4 % expects 4 numeric values on this dataline
            nodeID(nodeCnt) = round(lineData(1));
            nodeEstimate(nodeCnt,:) = lineData(2:4);
        else
            fclose(fileID);
            coder.internal.error('nav:navalgs:factorgraph:InvalidDataLine', lineCnt);
        end
        
    elseif strcmp(token, 'EDGE_SE3:QUAT')
        edgeCnt = edgeCnt + 1;
        % EDGE_SE3:QUAT  eid1 eid2  dx dy dz  dqx dqy dqz dqw
        %                           I_11 I_12 I_13 I_14 I_15 I_16
        %                                I_22 I_23 I_24 I_25 I_26
        %                                     I_33 I_34 I_35 I_36
        %                                          I_44 I_45 I_46
        %                                               I_55 I_56
        %                                                    I_66
        lineData = sscanf(remainingLine, '%f');
        lineData = lineData(:)';
        if numel(lineData) == 30 % expects 30 numeric values on this dataline
            nodeIDPair(edgeCnt,:) = [round(lineData(1)), round(lineData(2))];
            measurement(edgeCnt,:) = [lineData(3:5), lineData(9), lineData(6:8)]; % MATLAB expects quaternion to be in [qw, qx, qy, qz]
            information(edgeCnt,:) = lineData(10:30);
        else
            fclose(fileID);
            coder.internal.error('nav:navalgs:factorgraph:InvalidDataLine', lineCnt);
        end

    elseif strcmp(token, 'VERTEX_SE3:QUAT')
        nodeCnt = nodeCnt + 1;
        % VERTEX_SE3:QUAT vid x y z qx qy qz qw
        lineData = sscanf(remainingLine, '%f');
        lineData = lineData(:)';
        if numel(lineData) == 8 % expects 8 numeric values on this dataline
            nodeID(nodeCnt) = round(lineData(1));
            nodeEstimate(nodeCnt,:) = [lineData(2:4), lineData(8), lineData(5:7)];  % MATLAB expects quaternion to be in [qw, qx, qy, qz]
        else
            fclose(fileID);
            coder.internal.error('nav:navalgs:factorgraph:InvalidDataLine', lineCnt);
        end
    end
    nextLine = fgetl(fileID);
end
fclose(fileID);

% Create the factor graph

G = factorGraph;
% add factors
for i = 1 : size(nodeIDPair,1)
    
    if dim == 2
        f = factorTwoPoseSE2(nodeIDPair(i,:));
        f.Measurement = measurement(i,:);
        f.Information = robotics.core.internal.SEHelpers.deserializeInformationMatrixSE2(information(i,:));
    else
        f = factorTwoPoseSE3(nodeIDPair(i,:));
        f.Measurement = [measurement(i,:)];
        f.Information = robotics.core.internal.SEHelpers.deserializeInformationMatrixSE3(information(i,:));
    end 

    G.addFactor(f);
end

% add node initial guesses
for j = 1:size(nodeID,1)
    G.nodeState(nodeID(j), [nodeEstimate(j,:)]);
end

end
