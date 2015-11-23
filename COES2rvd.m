function [rvect,vvect]=...
    COES2rvd(a,ecc,inc,raan, omega,theta)

%compute r and v from COES; all angles in degrees
mu=398600;

theta = theta*pi/180;
omega = omega*pi/180;
raan = raan*pi/180;

%T = (2*pi/sqrt(mu))*a^(1.5);

%energy = -mu/(2*a);

%n = mu^(0.5)/a^(1.5);

p = a*(1-ecc^2);

h= (mu*p)^(1/2);

%E = 2*atan(sqrt((1-ecc/1+ecc))*tan(theta/2));

%book step
rvectx = (h^2/mu)*(1/(1+ecc*cos(theta))).*[cos(theta);sin(theta);0];
vvectx = (mu/h).*[-sin(theta);ecc+cos(theta);0];

%matrix conversion back into geocentric
term1 =[cos(omega) sin(omega) 0;...
        -sin(omega) cos(omega) 0; 0 0 1];
term2 =[1 0 0; 0 cosd(inc) sind(inc); 0 -sind(inc) cosd(inc)];
term3 =[cos(raan) sin(raan) 0;...
        -sin(raan) cos(raan) 0; 0 0 1];
    
convmat = term1*term2*term3;
invconvmat = inv(convmat);

rvect = invconvmat*rvectx;
vvect = invconvmat*vvectx;
end