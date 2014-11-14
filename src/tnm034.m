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
imshow(atImage);
atImage = medfilt2(atImage,[3 3]);


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

%Find center points

%Hopefully we found 3 components which match our criteria
[i, j] = find(L);
size(i,1)
size(j,1)

%Divide into grouped segments
connected = bwlabel(L,4);

[r, c] = find(connected == 1);
rc = [r c];
P1 = mean(rc);

[r, c] = find(connected == 2);
rc = [r c];
P2 = mean(rc);

[r, c] = find(connected == 3);
rc = [r c];
P3 = mean(rc);

figure;
imshow(I);
hold on;
plot(P1(2), P1(1), 'o');
plot(P2(2), P2(1), 'o');
plot(P3(2), P3(1), 'o');










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





strout=char(im);
strout = L;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%