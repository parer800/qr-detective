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
% imshow(atImage);
atImage = medfilt2(atImage,[3 3]);
% figure;
% imshow(atImage);


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

% figure;
% imshow(atImage);
% hold on;
% size(qr_locations_horizontal,2)
% for i=1:size(qr_locations_horizontal,2)
%     plot(qr_locations_horizontal(1,i),qr_locations_horizontal(2,i), '+');
% end
% 
% size(qr_locations_vertical,2)
% for i=1:size(qr_locations_vertical,2)
%     plot(qr_locations_vertical(1,i),qr_locations_vertical(2,i), '+');
% end

% figure;
% imshow(result_image_horizontal);
% figure;
% imshow(result_image_vertical);

combined = result_image_horizontal + result_image_vertical;
figure;
imshow(combined);

L = medfilt2(combined,[3 3]);
figure, imshow(L)

K = filter2(fspecial('average',3),combined); % <--- Not used

%Hopefully we found 3 components which match our criteria
[i, j] = find(L)
size(i,1)
size(j,1)




%================================================================
%                   SELECT 3 BIGGEST REGIONS
%================================================================
[P1, P2, P3] = find_FIP(L);

figure;
imshow(I);
hold on;
plot(P1(2), P1(1), 'ro');
plot(P2(2), P2(1), 'go');
plot(P3(2), P3(1), 'bo');


%================================================================
%                   ROTATE BY FIP POINTS [P1 P2 P3]
%================================================================

[Irotate, Prot] = rotate_qr(BW, P1, P2, P3);
Prot

figure;
imshow(Irotate);
hold on;
plot(Prot(1,2), Prot(1,1), 'ro');
plot(Prot(2,2), Prot(2,1), 'go');
plot(Prot(3,2), Prot(3,1), 'bo');



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
%                   RESIZE RATIO, Not necessary, can use pixel per block in
%                   both row and col direction.
%================================================================
xLength = Pcorner3(2) - Pcorner1(2)
yLength = Pcorner2(1) - Pcorner1(1)
ratio = xLength/yLength


if(ratio > 1)
    %X is larger, scale Y
    ratio = [size(Irotate,1)*ratio, size(Irotate,2)]
elseif(ratio < 1)
    %Y is larger, scale X
    ratio = [size(Irotate,1), size(Irotate,2)*ratio]
end
%Irotate = imresize(Irotate, ratio, 'nearest');
% figure;
% imshow(Irotate);



%================================================================
%                   FIND AP MARK
%================================================================
APTemplate = getTemplateOfAP(yLength, xLength);
% figure;
% imshow(APTemplate);
centerX = ceil((Pcorner3(2) - Pcorner2(2))/2);
centerY = ceil((Pcorner2(1) - Pcorner3(1))/2);
w = size(Irotate,2);
h = size(Irotate,1);

%AP = normxcorr2(APTemplate, Icrop(ceil(centerY):size(Icrop,1), ceil(centerX):size(Icrop, 2)));
%AP = normxcorr2(APTemplate, Irotate(centerY:floor(centerY + yLength), centerX:floor(centerX + xLength)));
figure;
imshow(Irotate(centerY:h, centerX:w));
AP = normxcorr2(APTemplate, Irotate(centerY:h, centerX:w));
[i j] = find(AP==max(max(AP)));
i = i + centerY;
j = j + centerX;
APpos = [i j]'
%APpos = APpos + Pcorner1 - [size(APTemplate,1)/2; size(APTemplate,1)/2]
offsetAPX = size(APTemplate,2)/2;
offsetAPY = size(APTemplate,1)/2;
APpos = APpos - [offsetAPY; offsetAPX];
figure;
imshow(AP)

%================================================================
%                   PERSPECTIVE DISTORTION
%================================================================


%Calculate fixed points
%Start with P1_ _ _ _ P3
%           |
%           | 
%           |     
%           P2        
%
PPB = (Pcorner3(2) - Pcorner1(2))/41 % Pixel per block
PPBX = (Pcorner3(2) - Pcorner1(2))/41 % Pixel per block in x
PPBY = (Pcorner2(1) - Pcorner1(1))/41 % Pixel per block in y

% BY FIP MARK
P1f = round(Prot(1,:))';
P2f = [P1f(1) + PPBY*(41-7); P1f(2)];
P3f = [P1f(1); P1f(2) + PPBX*(41-7)];
PAPf = round([P2f(1)-7*PPBY + PPBY/2;
        P3f(2)-7*PPBX + PPBX/2
        ]);

fipPoints = [P1f P2f P3f];
    
%BY CORNER POINTS     
P1f = Pcorner1
P2f = [P1f(1) + PPBY*41; P1f(2)]
P3f = [P1f(1); P1f(2) + PPBX*41];
PAPf = round([P2f(1)-7*PPBY + PPBY/2;
        P3f(2)-7*PPBX + PPBX/2
        ])
   

PAPf = round( [ P2f(1)-3.5*PPBY; P3f(2)-3.5*PPBX ]);
 
P1f = flip(P1f);
P2f = flip(P2f);
P3f = flip(P3f);
PAPf = flip(PAPf);



P4f = [P2f(1) P3f(2)]
%fixedPoints = [P1f P2f P3f PAPf]'
%movingPoints = [Pcorner1 Pcorner2 Pcorner3 APpos]'

fixedPoints = round([P1f P2f P3f PAPf]')
movingPoints = [round(Prot(1,:))' round(Prot(2,:))' round(Prot(3,:))' APpos]
movingPoints = flip(movingPoints)'

cornerPoints = flip([Pcorner1, Pcorner2, Pcorner3])'
fipPoints = flip(fipPoints)'
%tform = fitgeotrans(movingPoints, fixedPoints, 'projective')
%[Iwarp, RB] = imwarp(Irotate, tform, 'linear', 'outputview', imref2d(size(Irotate)), 'fillvalues', 1);
%RB

tform = cp2tform(movingPoints, fixedPoints, 'projective');
predicted_corner_points = tformfwd(tform, cornerPoints)
predicted_fip_points = tformfwd(tform, fipPoints)
correctCornerPoints = flip(predicted_corner_points')'
correctFipPoints = flip(predicted_fip_points')'
[row, col] = size(Irotate);
Iwarp = imtransform(Irotate, tform, 'XData', [1 col], 'YData', [1 row]);

P1f = flip(P1f);
P2f = flip(P2f);
P3f = flip(P3f);
PAPf = flip(PAPf);
movingPoints = flip(movingPoints')
figure;
imshow(Irotate);
hold on;

plot(j-offsetAPX,i-offsetAPY,'g+');
plot(PAPf(2),PAPf(1),'r+');
%plot(Pcorner1(2), Pcorner1(1), 'r+');
%plot(Pcorner2(2), Pcorner2(1), 'g+');
%plot(Pcorner3(2), Pcorner3(1), 'b+');

plot(Prot(1,2), Prot(1,1), 'g+');
plot(Prot(2,2), Prot(2,1), 'g+');
plot(Prot(3,2), Prot(3,1), 'g+');


plot(Pcorner1(2), Pcorner1(1), 'go');
plot(Pcorner2(2), Pcorner2(1), 'go');
plot(Pcorner3(2), Pcorner3(1), 'go');



plot(P4f(2),P4f(1),'b+');
plot(P1f(2),P1f(1),'ro');
plot(P2f(2),P2f(1),'ro');
plot(P3f(2),P3f(1),'ro');



figure;
imshow(Iwarp);
hold on;
plot(P1f(2),P1f(1),'b+');
plot(P2f(2),P2f(1),'b+');
plot(P3f(2),P3f(1),'b+');
plot(j-offsetAPX,i-offsetAPY,'g+');
plot(PAPf(2),PAPf(1),'r+');


plot(correctCornerPoints(1,2),correctCornerPoints(1,1),'r+');
plot(correctCornerPoints(2,2),correctCornerPoints(2,1),'g+');
plot(correctCornerPoints(3,2),correctCornerPoints(3,1),'b+');

Pcorner1 = flip([correctCornerPoints(1,2),correctCornerPoints(1,1)])
Pcorner2 = flip([correctCornerPoints(2,2),correctCornerPoints(2,1)])
Pcorner3 = flip([correctCornerPoints(3,2),correctCornerPoints(3,1)])

%set in P4f instead of ,..


%================================================================
%                   FIND CORNER POINTS AGAIN
%================================================================
%If image had perspective distortion, the corner points are not 
% optimal to crop with!
figure;
imshow(Iwarp);
hold on;


Pcorner1 = locate_corners(Iwarp, P1f', [-1, -1]);
plot(P1f(2), P1f(1), 'r+');

Pcorner2 = locate_corners(Iwarp, P2f', [-1, 1]);
plot(P2f(2), P2f(1), 'g+');

Pcorner3 = locate_corners(Iwarp, P3f', [1, -1]);
plot(P3f(2), P3f(1), 'b+');



%================================================================
%                   CROP IMAGE BY CORNER POINTS
%================================================================
xlength = Pcorner3(2) - Pcorner1(2)
ylength = Pcorner2(1) - Pcorner1(1)

cropRange = [Pcorner1(2) Pcorner1(1) xlength ylength]
Iwarp = imcrop(Iwarp, cropRange);
figure;
imshow(Iwarp);
hold on;
plot(P4f(2)-P1f(2),P4f(1)-P1f(1),'b+');
plot(P1f(2)-P1f(2),P1f(1)-P1f(1),'b+');
plot(P2f(2)-P1f(2),P2f(1)-P1f(1),'b+');
plot(P3f(2)-P1f(2),P3f(1)-P1f(1),'b+');
%plot(j-offsetAPX,i-offsetAPY,'g+');
%plot(PAPf(2),PAPf(1),'r+');



I2 = Iwarp
figure;
imshow(I2);


%================================================================
%                   TRANSLATE QR CODE TO STRING
%================================================================
level = graythresh(I2);
Iwarp = im2bw(I2, level);
finalstring = translate_qr(Iwarp);


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