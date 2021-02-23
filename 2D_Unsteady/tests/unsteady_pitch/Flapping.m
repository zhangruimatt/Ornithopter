addpath('../../');

% 2409, flapping, pitching
% User input: airfoil, onset flow, kinematics
foil = foil_naca4('2409',100,true);
Uinf = [cosd(0),sind(0)];           % onset flow [ux, uy]
foil.setPitchAxisOnChordLine(0);   % pitch axis position on the chordline
k = 1;                  % reduced frequency, pi*f*c/U_infty
alpmax = 5*pi/180;    % max angle of attack
nosc = 4;               % number of periods of oscillations in the whole process 
res = 100;               % number of steps

% Kinematics based on inputs
spdinf = norm(Uinf);
tau = foil.chord/spdinf;   % convective time
T = tau/k;              % period of oscillation
om = 2*pi/T;            % angular frequency
dt = 0.1*min(T,tau)/res;    % unsteady: dt/T small; convection: dt/tau small
tmax = nosc*T;          % max simulation time
t = 0:dt:tmax;          % discrete time steps
alp = zeros(size(t));   % pitch angle
x = zeros(size(t));     % surge position
amp = 0.2;              % amplitude of the oscillation (half peak-to-peak)
y = amp*sin(om*t);      % heave position 

% Create unsteady stepper and output files
stepper = UBEMStepper2d(foil,Uinf);

% Perform initial steady-flow step
fprintf('Initial steady-flow step ... ');
[Cp,xp,yp] = stepper.step();
[CFx,CFy,Cm] = forcemoment2d(foil,Cp,foil.chord,xp,yp,foil.puccw);
[Cd,Cl] = aerocoef2d(CFx,CFy,Uinf);

% Evaluate acceleration.
Fx = CFx * foil.chord;
Fy = CFy * foil.chord;
foil.ax = Fx/foil.m;
foil.ay = Fy/foil.m;
foil.vx = foil.ax*dt; 
foil.vy = foil.ay*dt; 
fprintf('done\n');

% Unsteady loop
nsteps = length(t)-1;
CD = zeros(1,nsteps+1);
CL = zeros(1,nsteps+1);
CM = zeros(1,nsteps+1);
Ein = zeros(1,nsteps+1);
Eout = zeros(1,nsteps+1);
CD(1) = Cd;
CL(1) = Cl;
CM(1) = Cm;

Vid = VideoWriter(sprintf('anim%.2f 2409.avi',amp));
Vid.Quality = 100;
Vid.FrameRate = 40;
open(Vid);

for i=1:nsteps
    fprintf('Unsteady step %d of %d ... ',i,nsteps);
    dalp = alp(i+1)-alp(i);
    dx = x(i+1)-x(i);
    dy = y(i+1)-y(i);
    dt = t(i+1)-t(i);
    [Cp,xp,yp] = stepper.step(dalp,dx,dy,dt);

    fprintf('done; circulation=%g\n',stepper.circt);
    
    % Update pitch axis position and compute aerodynamic coefficients
    [CFx,CFy,Cm] = forcemoment2d(foil,Cp,foil.chord,xp,yp,foil.puccw);
    % Evaluate acceleration.
    Fx = CFx * foil.chord;
    Fy = CFy * foil.chord;
    foil.ax = Fx/foil.m;
    foil.ay = Fy/foil.m;
    foil.vx = foil.vx + foil.ax*dt;
    foil.vy = foil.vy + foil.ay*dt;
    
    [Cd,Cl] = aerocoef2d(CFx,CFy,Uinf);
    CD(i+1) = Cd;
    CL(i+1) = Cl;
    CM(i+1) = Cm;
    Ein(i+1) = -(CFy*dy + Cm*dalp);
    Eout(i+1) = -(CFx*spdinf*dt); 
    
    shape = fill(-foil.x/foil.chord,foil.y/foil.chord,[.75 .75 .75]);
    hold on;

ip = find(stepper.wake.nu > 0);
in = find(stepper.wake.nu < 0);
p1 = plot(-stepper.wake.x(ip)/foil.chord,stepper.wake.y(ip)/foil.chord,'.r',...
    'MarkerSize',10);
p2 = plot(-stepper.wake.x(in)/foil.chord,stepper.wake.y(in)/foil.chord,'.b',...
    'MarkerSize',10);
set(gca,'FontSize',14);
axis('equal');
xlim([-5 4])
ylim([-2 7])

ff = getframe(gcf);
writeVideo(Vid,ff);
pause(0.001)

hold off
delete(p1);delete(p2);delete(shape);
    
end
Vid.close();    

I = 3:length(t);
fprintf('Efficiency = %g\n',sum(Eout(I))./sum(Ein(I)));

figure;
hold on;
subplot(3,1,1);
plot(t,alp,'LineWidth',1.5);
ylabel('$\alpha$','Interpreter','latex');
title('Kinematics','Interpreter','latex');
set(gca,'FontSize',14);
grid on;
subplot(3,1,2);
plot(t,x/f.chord,'LineWidth',1.5);
ylabel('$x/c$','Interpreter','latex');
set(gca,'FontSize',14);
grid on;
subplot(3,1,3);
plot(t,y/f.chord,'LineWidth',1.5);
ylabel('$y/c$', 'Interpreter', 'latex');
xlabel('$t/T$','Interpreter','latex');
set(gca,'FontSize',14);
grid on;
% saveas(gcf,'plots/kinematics.pdf');

figure;
hold on;
fill(f.x/f.chord,f.y/f.chord,[.9 .9 .9]);
%xlabel('$x/c$','Interpreter','latex');
%ylabel('$y/c$','Interpreter','latex');
ip = find(stepper.wake.nu > 0);
in = find(stepper.wake.nu < 0);
plot(stepper.wake.x(ip)/f.chord,stepper.wake.y(ip)/f.chord,'.r',...
    'MarkerSize',10);
plot(stepper.wake.x(in)/f.chord,stepper.wake.y(in)/f.chord,'.b',...
    'MarkerSize',10);
[~,cx,cy] = stepper.wake.vortexCores();
plot(cx,cy,'sk','MarkerSize',16);
set(gca,'FontSize',14);
axis('equal');
grid on;

figure;
hold on;
plot(t(I)/T,CL(I),t(I)/T,CM(I),t(I)/T,10*CD(I),'LineWidth',1.5);
xlabel('$t/T$','Interpreter','latex');
legend({'$C_L$','$C_M$','$10 C_D$'},'Interpreter','latex');
set(gca,'FontSize',14);
grid on;


figure;
subplot(2,1,1);
hold on;
fill(-f.x/f.chord,f.y/f.chord,[.75 .75 .75]);
ip = find(stepper.wake.nu > 0);
in = find(stepper.wake.nu < 0);
plot(-stepper.wake.x(ip)/f.chord,stepper.wake.y(ip)/f.chord,'.r',...
    'MarkerSize',10);
plot(-stepper.wake.x(in)/f.chord,stepper.wake.y(in)/f.chord,'.b',...
    'MarkerSize',10);
set(gca,'FontSize',14);
axis('equal');
axis('off');

subplot(2,1,2);
hold on;
tt=t(I)/T;
plot(tt,CL(I),'k-',tt,CM(I),'k--',tt,10*CD(I),'k-.','LineWidth',1.5);
text(4.05,1.25,'$C_L$','Interpreter','Latex','FontSize',14);
text(4.05,0.25,'$C_M$','Interpreter','Latex','FontSize',14);
text(4.05,-.75,'$C_D$','Interpreter','Latex','FontSize',14);
axis('off');


