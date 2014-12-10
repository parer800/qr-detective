function [ P1, P2, P3 ] = find_FIP( L )
%FIND_FIP find the three FIP points from the labled image L

connected = bwlabel(L);


D = regionprops(connected, 'Area')
area_for_regions = zeros(2, size(D,1));

for i=1:size(D,1)
    area_for_regions(:,i) = [D(i).Area, i]'; 
end

if(size(D,1) > 3)
    [a1, a2] = sort(area_for_regions(1,:), 'descend');
    sorted_areas = area_for_regions(:,a2)
else
    sorted_areas = area_for_regions
end

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



end

