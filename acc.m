function dx = acc(t,x)
m = 1650;
f0 = 0.1; f1 = 5; f2 = 0.25;
dx = zeros(5,1); 
u = x(4); %control input
v0 = x(5); %front vehicle speed
dx(1) = x(2);
dx(2) = (1/m)*(u - f0 - f1*x(2) - f2*x(2).^2); %dx(2) = (1/m)*u(1); without resistance
dx(3) = v0 - x(2);
dx(4) = 0; %piecewise constant control
dx(5) = 0; %front vehicle speed does not change during t