clear all;
clc;

%% Open Images, Crop nad save

%sat = imread('test.tif');
%sat = imread('senzaFiltri.tif');
sat3 = imread('3.tif');
sat2 = imread('2.tif');
%figure(1)
%imshow(sat2);
%figure(2) 
%imshow(sat3);

xmin = 3529 + 2000;
xmax = 7075;
ymin = 9282 -2000;
ymax = 14670;
widthCrop = abs(xmax-xmin) + 2000;
heightCrop = abs(ymax - ymin) - 2500;

I2 = imcrop(sat2,[xmin ymin widthCrop heightCrop]);
figure(4)
imshow(I2)
saveas(4, 'area1', 'tif');

xmin3 = 3107;
xmax3 = 7089;
ymin3 = 9582;
ymax3 = 14970;
widthCrop3 = abs(xmax3-xmin3);
heightCrop3 = abs(ymax3 - ymin3);

I3 = imcrop(sat3,[xmin3 ymin3 widthCrop3 heightCrop3]);
figure(5)
imshow(I3)
saveas(5, 'area3', 'tif');

%% Analyse and find features
% Use I3 (safe) and I2 (flood)

se = strel('disk', 20);
Io = imopen(I2, se);

% Nice results
Ie = imerode(I2, se);
Iobr = imreconstruct(Ie, I2);
figure
imshow(Iobr), title('Opening-by-reconstruction (Iobr)')

%% Blur 

Ioc = imclose(Io, se);

Iobrd = imdilate(Iobr, se);
Iobrcbr = imreconstruct(imcomplement(Iobrd), imcomplement(Iobr));
Iobrcbr = imcomplement(Iobrcbr);
figure
imshow(Iobrcbr), title('Opening-closing by reconstruction (Iobrcbr)')

%% Black and white

bw = im2bw(Iobrcbr, graythresh(Iobrcbr));
figure
imshow(bw), title('Thresholded opening-closing by recon');

bw = im2bw(Iobrcbr, graythresh(Iobrcbr));
imshow(bw)


%% Get Centroids and radius of convex hull
figure(10)
bwInv = imcomplement(bw);
imshow(bwInv)

stats = regionprops(bwInv,'Centroid','MajorAxisLength','MinorAxisLength',...
    'Orientation','Eccentricity','Extrema','BoundingBox','ConvexArea',...
    'ConvexHull','ConvexImage')

% Plot centroids in negative image
centroids = cat(1, stats.Centroid);
imshow(bwInv)
hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off

%% 
% Plot centroids in convex sectors
% h = figure(5)
% CH_objects = bwconvhull(bwInv,'objects',8);
% imshow(CH_objects);
% hold on
% 
% plot(centroids(:,1),centroids(:,2), 'b*')
% hold off
% title('Objects Convex Hull + centroids');
    
%% Plot elliptic approximation

%imshow(BW)
hold on


phi = linspace(0,2*pi,50);
cosphi = cos(phi);
sinphi = sin(phi);

for k = 1:length(stats)
    xbar = stats(k).Centroid(1);
    ybar = stats(k).Centroid(2);

    a = stats(k).MajorAxisLength/2;
    b = stats(k).MinorAxisLength/2;

    theta = pi*stats(k).Orientation/180;
    R = [ cos(theta)   sin(theta)
         -sin(theta)   cos(theta)];

    xy = [a*cosphi; b*sinphi];
    xy = R*xy;

    x = xy(1,:) + xbar;
    y = xy(2,:) + ybar;

    plot(x,y,'r','LineWidth',2);

end
hold off

% %% Plot and save extrema
% figure(5)
% hold on
% for secteur = 1:size(stats,1)    
%     for i = 1:8
%         plot(stats(secteur,1).Extrema(i,1),stats(secteur,1).Extrema(i,2),'gO')
%         hold on
%     end
%     hold on
% end

%%  Repeat for normal records
% ind2gray(I3,map);
% imshow(I3,map)
% figure,imshow(I3)
% 
% K = mat2gray(I3);
% imshow(K)
% 
% se3 = strel('disk', 20);
% Io3 = imopen(I3, se3);
% 
% % Nice results
% Ie3 = imerode(I3, se3);
% Iobr3 = imreconstruct(Ie3, I3);
% %figure(11)
% %imshow(Iobr3), title('Opening-by-reconstruction (Iobr)')
% 
% %% 
% 
% Ioc = imclose(Io3, se3);
% 
% Iobrd = imdilate(Iobr3, se3);
% Iobrcbr3 = imreconstruct(imcomplement(Iobrd), imcomplement(Iobr3));
% Iobrcbr3 = imcomplement(Iobrcbr);
% figure
% imshow(Iobrcbr), title('Opening-closing by reconstruction (Iobrcbr)')
% 
% % Black and white
% 
% bw3 = im2bw(Iobrcbr, graythresh(Iobrcbr));
% figure
% imshow(bw3), title('Thresholded opening-closing by recon');
% 
% bw3 = im2bw(Iobrcbr, graythresh(Iobrcbr));
% imshow(bw3)
% 
% 
% %% Get Centroids and radius of convex hull
% figure(10)
% bwInv = imcomplement(bw3);
% imshow(bwInv)
% 
% stats = regionprops(bwInv,'Centroid','MajorAxisLength','MinorAxisLength',...
%     'Orientation','Eccentricity','Extrema','BoundingBox','ConvexArea',...
%     'ConvexHull','ConvexImage')
% 
% % Plot centroids in negative image
% centroids = cat(1, stats.Centroid);
% imshow(bwInv)
% hold on
% plot(centroids(:,1),centroids(:,2), 'b*')
% hold off
% 
% % Plot elliptic approximation
% 
% %imshow(BW)
% hold on
% 
% phi = linspace(0,2*pi,50);
% cosphi = cos(phi);
% sinphi = sin(phi);
% 
% for k = 1:length(stats)
%     xbar = stats(k).Centroid(1);
%     ybar = stats(k).Centroid(2);
% 
%     a = stats(k).MajorAxisLength/2;
%     b = stats(k).MinorAxisLength/2;
% 
%     theta = pi*stats(k).Orientation/180;
%     R = [ cos(theta)   sin(theta)
%          -sin(theta)   cos(theta)];
% 
%     xy = [a*cosphi; b*sinphi];
%     xy = R*xy;
% 
%     x = xy(1,:) + xbar;
%     y = xy(2,:) + ybar;
% 
%     plot(x,y,'r','LineWidth',2);
% 
% end
% hold off

%% Final result

h = figure(30)
imshow(bwInv)
hold on
figure
set(0,'CurrentFigure',h);
voronoi(centroids(:,1),centroids(:,2))

set(0,'CurrentFigure',bwFig);
voronoi(centroids(:,1),centroids(:,2))

hold off


