function [occMatrix] = PyGetMap()
%PYGETMAP Summary of this function goes here
%   Detailed explanation goes here
    global map

    occMatrix = getOccupancy(map);
end

