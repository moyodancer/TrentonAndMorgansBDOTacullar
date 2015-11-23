function [ pq ] = quatProd( p, q )
%should come in as columns
pq = zeros(4, 1);
Epcross= [0 -p(4) p(3); p(4) 0 -p(2); -p(4) p(2) 0];

pq(1,1) = p(1)*q(1)- dot(p(2:4), q(2:4));
pq(2:4, 1) = p(1)*q(2:4, 1) +q(1)*p(2:4, 1) + Epcross*q(2:4, 1);
end

