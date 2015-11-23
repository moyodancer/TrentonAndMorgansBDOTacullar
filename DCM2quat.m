function [ q ] = DCM2quat( c )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
eta = .5*sqrt(1+trace(c));
epsilon1 = .25*(c(2,3)-c(3,2))/eta;
epsilon2 = .25*(c(3,1)-c(1,3))/eta;
epsilon3 = .25*(c(1,2)-c(2,1))/eta;
q =[eta; epsilon1; epsilon2; epsilon3];

end

