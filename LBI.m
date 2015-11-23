function [ LBI ] = LBI( theta, phi)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
LBI = (1/cos(theta))*[cos(theta) sin(phi)*sin(theta) cos(phi)*sin(theta);...
    0 cos(phi)*cos(theta) -sin(phi)*cos(theta);...
    0 sin(phi) cos(phi)];

end

