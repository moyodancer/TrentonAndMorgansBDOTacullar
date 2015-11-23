function [ DCM ] = LVLH2ECI( rvect, vvect )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

x = [1 0 0];
y = [0 1 0];
z = [0 0 1];
h_a = cross(rvect, vvect);
i_hat = rvect/norm(rvect);
k_hat = h_a/norm(h_a);
j_hat = cross(k_hat, i_hat);
Q= [i_hat j_hat k_hat]; %inertial to lvlh
DCM = [dot(x, i_hat) dot(x, j_hat) dot(x, k_hat);
       dot(y, i_hat) dot(y, j_hat) dot(y, k_hat);
       dot(z, i_hat) dot(z, j_hat) dot(z, k_hat)];
end

