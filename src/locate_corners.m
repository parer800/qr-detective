function [ output_args ] = locate_corners(I, P, dir )
%LOCATE_CORNERS Summary of this function goes here
%   dir(x,y) direction of x and y
    level = graythresh(I);
    I = im2bw(I, level);

    colorSwitchWanted = 3;
    x = round(P(1,2));
    y = round(P(1,1));

    %Check distance in x
    while colorSwitchWanted > 0
        if(I(y,x) ~= I(y,x+1*dir(1)))
            colorSwitchWanted = colorSwitchWanted - 1;
        end
        x = x + 1*dir(1);
    end
    x = x+1*(-1*dir(1)); % stepped one to far
    %check distance in y
    colorSwitchWanted = 3;
    xstart = round(P(1,2));
    while colorSwitchWanted > 0
        if(I(y,xstart) ~= I(y+1*dir(2),xstart))
            colorSwitchWanted = colorSwitchWanted - 1;
        end
        y = y + 1*dir(2);
    end
    y = y+1*(-1*dir(2)); % stepped one to far
    output_args = [y; x];


end

