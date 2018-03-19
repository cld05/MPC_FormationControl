function dx = ode(x,u,t)

dx(1,1) = x(2);
dx(2,1) = u;