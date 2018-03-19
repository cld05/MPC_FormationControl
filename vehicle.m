function out = vehicle(states,control,Ts)

% A = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
% B = [0.005 0; 0 0.005; 0.1 0; 0 0.1];

A  = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0]; B = [0 0; 0 0; 1 0; 0 1];

out = A*states + B*control;





% states = [x(1,i), y(1,i), dx(1,i), dy(1,i)].'
% control = u(:,i)






