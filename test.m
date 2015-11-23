cc = 0;
a = 42000; %km
inc = 0; %degrees
raan = 0; %degrees
omega = 0; %degrees
theta = 0;
mu = 398600; %km^2 something
[rvect,vvect]=COES2rvd(a,ecc,inc,raan, omega, theta);
rvect = rvect';
vvect = vvect';
h_a = cross(rvect,vvect);
k_hat = -rvect/norm(rvect);
j_hat = -h_a/norm(h_a);
i_hat = cross(j_hat, k_hat);
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];
c = [dot(x, i_hat) dot(x, j_hat) dot(x, k_hat);
       dot(y, i_hat) dot(y, j_hat) dot(y, k_hat);
       dot(z, i_hat) dot(z, j_hat) dot(z, k_hat)];
out = DCM2quat(c)