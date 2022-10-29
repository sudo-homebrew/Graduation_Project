function [ids, time, lla, vned, nacp, nacv, gva, attributes ] = parseMessage(messages)
%PARSEMESSAGE Parses through the fields of an ADS-B message structure 
% Inputs:
%   messages      - ADS-B message struct array
% Outputs:
%   ids           - ICAO identifiers
%   time          - Message timestamps
%   lla           - Geodetic positions
%   vned          - Local velocity vectors (NED)
%   nacp          - Navigation Accuracy Category Position
%   nacv          - Navigation Accuracy Category Velocity
%   gva           - Geometric Vertical Accuracy categories
%   attributes    - ADS-B attributes
%
% This is an internal function and may be removed or modified in a future
% release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

numMessages = numel(messages);
ids = cell(1,numMessages);
time = zeros(1,numMessages);
lla = zeros(3, numMessages);
vned = zeros(3, numMessages);
nacp = zeros(1, numMessages);
nacv = zeros(1, numMessages);
gva = zeros(1, numMessages);
attributes = cell(1, numMessages);
for i=1:numMessages
    message = messages(i);
    ids{i} = message.ICAO;
    time(i) = message.Time;
    lla(:,i) = [message.Latitude; message.Longitude; message.Altitude];
    vned(:,i) = [message.Vnorth ; message.Veast ; - message.ClimbRate];
    nacp(i) = message.NACPosition;
    nacv(i) = message.NACVelocity;
    gva(i) = message.GeometricVerticalAccuracy;
    attr.Callsign = message.Callsign;
    attr.Category = message.Category;
    attributes{i} = attr;
end

end