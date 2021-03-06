XMin = -10;
XMax = 10;
YMin = -10;
YMax = 10;

%% 3D function RBF
disp('3D case:');
[X,Y] = meshgrid(0:.1:10);
%center
Xc1=center*ones(101,202);
Yc1=center*ones(101,202);
Z = exp(-((X-Xc1(:,1:101)).^2+(Y-Yc1(:,102:202)).^2)*esp.^2);

% sigma1=.8; 
% esp1=1/(2*(sigma1)^2);
% center1=8;
% Xc2=center1*ones(101,202);
% Yc2=center*ones(101,202);
% Z = Z + exp(-((X-Xc2(:,1:101)).^2+(Y-Yc2(:,102:202)).^2)*esp1.^2);
surf(X,Y,Z)
%% Random Radial Basis functions in space 
disp('3D case with Random path - 3 functions:');
N = 8 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)

[cline]=contour(Z,[0.5 0.5]);

%pause();
%% Random Radial Basis functions in space 
disp('3D case with random path - 10 rbf:');
N = 10 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)

%% Post elaboration

figure(2);
grid on
axis equal
axis tight
contour(X,Y,Z);
h=[1,1]
contour(X,Y,Z,h);
cl = contour(X,Y,Z);
[x1,y1,z1] = C2xyz(cl);


figure(1);
hold on; % Allows plotting atop the preexisting peaks plot. 
threshold = 1;
sector.numberOfZones = 1;
% analyze all the level curves
for n = find(z1==threshold); 
   n
   %xTh(1,n) = x1{n};
   %yTh(1,n) = y1{n};
   plot(x1{n},y1{n},'k-','linewidth',3);
   sector.zones.x(sector.numberOfZones,:) = x1{n}(1,:);
   sector.zones.y(sector.numberOfZones,:) = y1{n}(1,:);
   sector.numberOfZones = sector.numberOfZones + 1;
end
disp('Number of zones founded:');
disp(numberOfZones);
plot(xTh(1,:),yTh(1,:),'k-','linewidth',3)
x1{83}(1,2)

%plot(cl(1,8:end),cl(2,8:end))

% v = -2:0.2:2;
% [x,y] = meshgrid(v);
% z = x .* exp(-x.^2 - y.^2);
% [px,py] = gradient(z,.2,.2);
% contour(v,v,z)
% hold on
% quiver(v,v,px,py)
% hold off

%% 3D function
disp('3D case:');
[X,Y] = meshgrid(-10:.1:10);
Z = -X.^2+-(Y).^2+100;
surf(X,Y,Z)
h=[1,1]
contour(X,Y,Z,h)

% Level Curve
syms x y;
soly = solve(10==-x.^2-y^2+100, y)
 x=-10:0.1:10;
 t=soly(1)
 figure;
 plot(x,(90 - x.^2).^(1/2));
 hold on
 plot(x,-(90-x.^2).^(1/2));
% y=-20:0.1:20;
% plot((-x.^2 - 1).^(1/2),(- y.^2 - 1).^(1/2))
%Level curve of a sphere z=f(x,y)
x0=3;
y0=5;
r=1;
r1=r/2;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x0+xp,y0+yp);
hold on
xp1=r1*cos(ang);
yp1=r1*sin(ang);
plot(x0+xp1,y0+yp1);


sola = solve(a*x^2 + b*x + c == 0, a)

syms y x;
soly = solve(6==x.^2+y^2, y)
solx = solve(6==x.^2+y^2, x)
x=0:0.1:10;
y=0;0.1:10;
yp=soly(1)
yp=0:0.1:10;
plot(xp,yp)


figure;
contour3(X,Y,Z);
grid on
box on
view([130,30])
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
title('Contours at height for random gaussians.')


xdata = [2 2 0 2 5;
         2 8 2 4 5;
         8 8 2 4 8];
ydata = [4 4 4 2 0;
         8 4 6 2 2;
         4 0 4 0 0];
zdata = ones(3,5);

%% Random Radial Basis functions in space
disp('3D case with random path - 500 RBF:');
N = 10 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)
pause();

%% Plane in 3D space
[X,Y] = meshgrid(95:0.1:105);
a=0.1;
b=0.2;
c=1;
d=0;
Z=(-a * X - b * Y)/c;
surf(X,Y,Z)
shading flat
xlabel('x')
ylabel('y')
zlabel('z')
pause();

%% TEst
figure;
x = linspace(-2*pi,2*pi);
y = linspace(0,4*pi);
[X,Y] = meshgrid(x,y);
Z = sin(X)+cos(Y);
surf(X,Y,Z);

figure;
[X,Y,Z] = peaks;
surf(X,Y,Z);
contour(X,Y,Z,20);

fig = figure;
[X,Y] = mehgrid(-8:.5:8);
R = sqrt(X.^2 + Y.^2) + eps;
Z = sin(R)./R;
surf(Z)
%p = fig2plotly(fig);
disp('End');