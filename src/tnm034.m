%%%%%%%%%%%%%%%%%%%%%%%%%% 
function strout = tnm034(im) 
% 
% Im: Input image of captured sheet music. Im should be in
% double format, normalized to the interval [0,1] 
% strout: The resulting character string of the coded 
% The string must have exactly the number of characters that 
% corresponds to the information in the code. 
%
% Your program code.
%

I = imread(im);

%Get dimensions, to check if 
[x,y,z] = size(I);
if z == 3
    z
    BW = rgb2gray(I);
    G = fspecial('disk',15);
    %Ig = imfilter(BW,G,'same');
    
    %BW = edge(BW, 'canny');
else
    z
    BW = I;
    %BW = edge(BW, 'canny');
end
%adaptive threshold image
atImage = adaptivethres(double(BW));
atImage_unfiltered = atImage;
imshow(atImage);
atImage = medfilt2(atImage,[3 3]);
figure;
imshow(atImage);


[H, T, R] = hough(atImage);


% Display the original image.
% subplot(2,1,1);
% figure;
% 
% title(im);

%LOOP THROUGH AND SAVE on every color switch, position (x,y), pixelcolor, counter. 
width = size(atImage,2);
height = size(atImage,1);


   
[result_image_horizontal, qr_locations_horizontal] = locate_qr(atImage, false);
[result_image_vertical, qr_locations_vertical] = locate_qr(atImage, true);

figure;
imshow(atImage);
hold on;
size(qr_locations_horizontal,2)
for i=1:size(qr_locations_horizontal,2)
    plot(qr_locations_horizontal(1,i),qr_locations_horizontal(2,i), '+');
end

size(qr_locations_vertical,2)
for i=1:size(qr_locations_vertical,2)
    plot(qr_locations_vertical(1,i),qr_locations_vertical(2,i), '+');
end

figure;
imshow(result_image_horizontal);
figure;
imshow(result_image_vertical);

combined = result_image_horizontal + result_image_vertical;
figure;
imshow(combined);

L = medfilt2(combined,[3 3]);
figure, imshow(L)

K = filter2(fspecial('average',3),combined);
figure, imshow(K)

%imshow(segment(:,ticker+4));


%[largest_cluster_area, largest_cluster_label] = max([D.Area])

%Find center points

%Hopefully we found 3 components which match our criteria
[i, j] = find(L)
size(i,1)
size(j,1)

%Divide into grouped segments
connected = bwlabel(L);


%================================================================
%                   SELECT 3 BIGGEST REGIONS
%================================================================

D = regionprops(connected, 'Area')
area_for_regions = zeros(2, size(D,1));

for i=1:size(D,1)
    area_for_regions(:,i) = [D(i).Area, i]'; 
end

area_for_regions
[a1, a2] = sort(area_for_regions(1,:), 'descend');
sorted_areas = area_for_regions(:,a2)



cc = bwconncomp(L);
s = regionprops(cc, 'PixelIdxList', 'Area')
areasss = s.Area

[r, c] = find(connected == sorted_areas(2,1));
rc = [r c];
P1 = mean(rc)'

[r, c] = find(connected == sorted_areas(2,2));
rc = [r c];
P2 = mean(rc)'

[r, c] = find(connected == sorted_areas(2,3));
rc = [r c]
P3 = mean(rc)'


%Check which point should be P1 and P2
P21 = P2-P1;
P31 = P3-P1;

P12 = P1-P2;
P32 = P3-P2;

P21x31 = P21' * P31;
P12x32 = P12' * P32;

%Lowest dot value (probably 0) of P21x31 is perpendicular to P3 which
%should be P1
if(P21x31 > P12x32)
    temp = P1;
    P1 = P2;
    P2 = temp;
end


figure;
imshow(I);
hold on;
plot(P1(2), P1(1), 'ro');
plot(P2(2), P2(1), 'go');
plot(P3(2), P3(1), 'bo');





P = (P3-P1)';
Pn = norm(P)
P
xaxis = [0 1]'

P * xaxis
px = P*xaxis
theta = acos((P*xaxis)/Pn);
thetaDegrees = rad2deg(theta)


Irotate = imrotate(BW, thetaDegrees);
%Local threshold instead of adaptive
level = graythresh(Irotate);
Irotate = im2bw(Irotate, level);

Irotate = adaptivethres(double(Irotate));
costheta = cos(theta);
sintheta = sin(theta);
rotmatrix = [costheta -sintheta; sintheta costheta]'
Ptot = [P1'; P2'; P3']


xmiddle = size(Irotate, 2)/2
ymiddle = size(Irotate, 1)/2

Ptot(:,2) = Ptot(:,2) - width/2;
Ptot(:,1) = Ptot(:,1) - height/2;
Ptot
Prot = Ptot * rotmatrix

Prot(:,2) = Prot(:,2) + xmiddle;
Prot(:,1) = Prot(:,1) + ymiddle;

figure;
imshow(Irotate);
hold on;
plot(Prot(1,2), Prot(1,1), 'ro');
plot(Prot(2,2), Prot(2,1), 'go');
plot(Prot(3,2), Prot(3,1), 'bo');

%Angle between P2


%================================================================
%                   FIND CORNER POINTS
%================================================================
Pcorner1 = locate_corners(Irotate, Prot(1,:), [-1, -1]);
plot(Pcorner1(2), Pcorner1(1), 'r+');

Pcorner2 = locate_corners(Irotate, Prot(2,:), [-1, 1]);
plot(Pcorner2(2), Pcorner2(1), 'g+');

Pcorner3 = locate_corners(Irotate, Prot(3,:), [1, -1]);
plot(Pcorner3(2), Pcorner3(1), 'b+');


%================================================================
%                   CROP IMAGE BY CORNER POINTS
%================================================================
xlength = Pcorner3(2) - Pcorner1(2)
ylength = Pcorner2(1) - Pcorner1(1)

Pcrop = [Pcorner1(1)+ylength, Pcorner1(2)+xlength]
cropRange = [Pcorner1(2) Pcorner1(1) xlength ylength]
Icrop = imcrop(Irotate, cropRange);
figure;
imshow(Icrop);

%================================================================
%                   TRANSLATE QR CODE TO STRING
%================================================================

finalstring = translate_qr(Icrop);


% Display & calculate the Hough matrix.
% 
% P  = houghpeaks(H,5);
% 
% subplot(2,1,2);
% imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
%       'InitialMagnification','fit');
% title('Hough Transform of Image');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% plot(T(P(:,2)),R(P(:,1)),'s','color','red');
% %colormap(hot);
% lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
% figure, imshow(atImage), hold on
% max_len = 0;
% for k = 1:length(lines)
%    xy = [lines(k).point1; lines(k).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% 
%    % Determine the endpoints of the longest line segment
%    len = norm(lines(k).point1 - lines(k).point2);
%    if ( len > max_len)
%       max_len = len;
%       xy_long = xy;
%    end
% end
% 
% % highlight the longest line segment
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','blue');





%strout=char(im);
strout = finalstring;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%