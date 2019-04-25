% Matlab programma ter voorspelling van de baan afgelegd door een
% gelanceerde pinpongbal. 
launchHight = 0.10;
maxDistance = 1;
maxHight = 0.5;
speed = 2;
angle = pi/4;

% Tweede orde differentiaalvergelijking
[t,y] = ode45(@vdp1,[0 maxDistance],[0; launchHight; (speed*cos(angle)); (speed*sin(angle))]);
plot(y(:,1),y(:,2),'-o')
xlabel('Afstand[m]')
ylabel('Hoogte[m]')
axis([0 maxDistance 0 maxDistance])


syms vx(x) vy(y) rho m A Cw g

ax = diff(vx);
ay = diff(vy);

odex = m*ax == 0.5*rho*vx*A*Cw;
odey = m*ay == 0.5*rho*vy*A*Cw - m*g;



solx = dsolve(odex)
soly = dsolve(odey)

x = 0;
y = launchHight;
vglx = subs(solx)
vgly = subs(soly)





 
 
function dydt = vdp1(t,y)
rho = 1.205;  % luchtdensiteit
r = 0.02;  % straal van pingpongbal
surface = pi * r^2;  % oppervlakte van pingpongbal
Cw = 0.4;  % luchtwrijvingscoëfficiënt
m = 0.0027;  % massa van pingpongbal
g = 9.81;  % valversnelling aards gravitatieveld

% ? stelsel van vier eersteorde differentiaalvergelijkingen ?
dydt= [(y(3)); 
    (y(4));
    (((-1/2 * rho * surface * Cw)/ m)*(y(3)*sqrt(y(3)^2 + y(4)^2)));
    (((-1/2 * rho * surface * Cw)/ m)*(y(4)*sqrt(y(3)^2 + y(4)^2)) - g)];
end












