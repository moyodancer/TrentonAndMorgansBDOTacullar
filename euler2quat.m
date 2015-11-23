function [ eta, epsilon ] = euler2quat( euler )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
thetax = euler(1);
thetay = euler(2);
thetaz = euler(3);
cx = [1 0 0;
      0 cos(thetax) sin(thetax);
      0 -sin(thetax) cos(thetax)];
cy = [cos(thetay) 0 -sin(thetay);
      0 1 0;
      sin(thetay) 0 cos(thetay)];
cz = [cos(thetaz) sin(thetaz) 0;
      -sin(thetaz) cos(thetaz) 0;
      0 0 1];
c = cx*cy*cz;

eta = .5*sqrt(1+trace(c));
epsilon(1) = .25*(c(2,3)-c(3,2))/eta;
epsilon(2) = .25*(c(3,1)-c(1,3))/eta;
epsilon(3) = .25*(c(1,2)-c(2,1))/eta;

end

