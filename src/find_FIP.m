function [ P1, P2, P3 ] = find_FIP( L )
%FIND_FIP find the three FIP points from the labled image L

connected = bwlabel(L);


D = regionprops(connected, 'Area');
area_for_regions = zeros(2, size(D,1));

for i=1:size(D,1)
    area_for_regions(:,i) = [D(i).Area, i]'; 
end
%area_for_regions
if(size(D,1) > 3)
    %sort from the 2 position backwards
    upper_row = area_for_regions(1,:);
    area_for_regions(1,:) = (upper_row-0.2*max(upper_row))/max(upper_row); % Normalize the values, remove 20% of the maxvalue to guarantee correct points
    area_for_regions(1,:) = round(area_for_regions(1,:));
    
    wanted_nr_points = 3;
    counter = 1;
    points = [];
    while(wanted_nr_points)
       if(area_for_regions(1,counter) == 1)
        points = [points [1;area_for_regions(2,counter)]];
        wanted_nr_points = wanted_nr_points-1;
       end
       counter = counter+1;
    
    end
    
    
    %[a1, a2] = sort(area_for_regions(1,3:length(area_for_regions)), 'descend');
    %a2 = a2 + 2;
    %area_for_regions(:,3:length(area_for_regions)) = area_for_regions(:,a2);
    sorted_areas = points;
else
    sorted_areas = area_for_regions;
end

[r, c] = find(connected == sorted_areas(2,1));
rc = [r c];
P1 = mean(rc)';

[r, c] = find(connected == sorted_areas(2,2));
rc = [r c];
P2 = mean(rc)';

[r, c] = find(connected == sorted_areas(2,3));
rc = [r c];
P3 = mean(rc)';

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

