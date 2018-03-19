clear all;
close all;
clc;
                     
T = 10; dt = 0.01; 
t = 0:dt:T;
n = length(t);
uL = zeros(2,n);
zL = zeros(4,n);
des_traj = zeros(4,n);
uF1 = zeros(2,n);
zF1 = zeros(4,n);


%% initial states
zL(1,1) = 0;
zL(2,1) = -10;
zL(3,1) = 0;
zL(4,1) = 0;

zF1(1,1) = -10;
zF1(2,1) = 10;
zF1(3,1) = 0;
zF1(4,1) = 0;

%% Leader's trajectory
vxL = 5;  vyL = 0; 
xd0 = 10; yd0 = 10;
des_traj(:,1)  = [xd0 yd0 vxL vyL].';


% x(:,1) = [100 0].';
% Run the test
for i = 1:n
    
    %% Leader
    outL = Leader_RUN(zL(1,i),zL(2,i),zL(3,i),zL(4,i),...   % own states
                     des_traj(1,i),des_traj(2,i),des_traj(3,i),des_traj(4,i)); % reference
    uL(:,i) = outL.CONTROLS(1,2:3).';
%     z(:,i) = out.STATES(1,2:5).';
    zL(:,i+1) = rk4('vehicle',zL(:,i),uL(:,i),dt);
    
    if i == 1
       Pred = outL.STATES; 
       U    = outL.CONTROLS;
    end
    
    %% Follower
    outF = Leader_RUN(zF1(1,i),zF1(2,i),zF1(3,i),zF1(4,i),...  % own states
                      zL(1,i),zL(2,i),zL(3,i),zL(4,i));        % reference
    uF1(:,i) = outF.CONTROLS(1,2:3).';
%     z(:,i) = out.STATES(1,2:5).';
    zF1(:,i+1) = rk4('vehicle',zF1(:,i),uF1(:,i),dt);
    
    %% Desired trajectory
    des_traj(:,i+1) = des_traj(:,i) + [vxL vyL 0 0].'*dt;
end

%% Figures
figure(1)
subplot(311)
plot(t,zL(1:2,1:n),'linewidth',2)
xlabel('t')
ylabel('pos')
grid on
subplot(312)
plot(t,zL(3:4,1:n),'linewidth',2)
xlabel('t')
ylabel('vel')
grid on
subplot(313)
plot(t,uL(:,1:n),'linewidth',2)
xlabel('t')
ylabel('control')
grid on

figure(2)
subplot(311)
plot(t,zF1(1:2,1:n),'linewidth',2)
xlabel('t')
ylabel('pos')
grid on
subplot(312)
plot(t,zF1(3:4,1:n),'linewidth',2)
xlabel('t')
ylabel('vel')
grid on
subplot(313)
plot(t,uF1(:,1:n),'linewidth',2)
xlabel('t')
ylabel('control')
grid on

figure(3)
hold on
plot(zL(1,1:n),zL(2,1:n),'linewidth',2)
plot(zF1(1,1:n),zF1(2,1:n),'linewidth',2)
grid on
axis equal
xlabel('x')
ylabel('y')

figure(4)
hold on
subplot(311)
plot(t,zL(1:2,1:n),'linewidth',2)
plot(Pred(:,1),Pred(:,2:3),'linewidth',2)
xlabel('t')
ylabel('pos')
grid on

subplot(312)
hold on
plot(t,zL(1:2,1:n),'linewidth',2)
% plot(Pred(:,1), Pred(:,2:3),'linewidth',2) % comparison predicted positions / real positions from MPC at the initial time
xlabel('t')
ylabel('vel')
grid on

subplot(313)
hold on
plot(t,uL(:,1:n),'linewidth',2)
% plot(U(:,1),U(:,2:3),'linewidth',2) % comparison predicted control / real control from MPC at the initial time
xlabel('t')
ylabel('control')
grid on
