function [ result_image, qr_locations ] = locate_qr( in, verticalDirection )
%LOCATE_QR Summary of this function goes here
%   Detailed explanation goes here
    if(verticalDirection)
        in = in'; % Transpose the image if we want to check vertically, instead of rewriting the loop for that case
        figure;
        imshow(in);
    end
    
    
    function A = swap_rows(A,i,j)
       % assert(i > 0 && i < size(A,1) && j > 0 && j < size(A,1) );
        TEMP = A([j i], :);
        A([i j], :) = TEMP; % Notice the difference of i & j
    end
    

    width = size(in,2);
    height = size(in,1);
    qr_locations = double.empty();
    result_image = zeros(height, width);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%% STORE COLOR SWITCHES %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Store in block of 4: [x,y,colorSeq, counterSeq]
    
    segment = -ones(4,height*width);
    color_switch = in(1,1);
    sequence_counter = 0;
    ticker = 1;
    for y=1:height
        %current_v = atImage(y,x);
        sequence_counter = 0;
        color_switch = in(y,1); % reset for every loop in Y-led
        for x=1:width
            %IF color has switched, store previous sequence in segment
            if in(y,x) ~= color_switch
                %Color has switched
                component = [x-1,y,color_switch, sequence_counter]';
                segment(:,ticker) = component;
                ticker = ticker+1;
                sequence_counter = 0;
                color_switch = in(y,x);
            else
                sequence_counter = sequence_counter + 1;
            end
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%% CHECK RATIO BETWEEN COLOR SWITCHES %%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:1:width*height
        if segment(1,i) == -1 || segment(1,i+4) == -1
            break;
        end
        sum = 0;
        first_color = segment(3, i);
        position = 0;
        for j=0:4
            sumcomponent(j+1) = segment(4,i+j);
            sum = sum + segment(4, i+j); % Get every counter value, add that to sum. check if modulus(sum, 7) is equal to 0 
        end

        %Get block of five which we look at [i, i+1, i+2, i+3, i+4]
        block = segment(:,i:i+4);
        centerIndex = 3;
        centerBlock = block(:,centerIndex); % Store centerblock to check ratio later
        [mValue, mIndex] = max(block(4,:)); % get max value of sequence_count and its index, max should be the centered value in the ratio sequence 1:1:[3]:1:1

        if(mIndex ~= centerIndex)
            continue; %continue if maxvalue is not centered
        end

        c = centerBlock(4);
        qr_flag = -1;

        for j=1:2
            l = block(4,centerIndex - j); %left value
            r = block(4,centerIndex + j); %right value
            lc = c/l; %center-left ratio
            rc = c/r; %center-right ratio
            cn = 3; %Normalized center weight
            faultPercentage = 1.5;

            cnlc = abs(cn-lc);
            cnrc = abs(cn-rc);

            if( cnlc > faultPercentage || cnrc > faultPercentage)
                qr_flag = -1;
                break;
            else
                qr_flag = 1;
            end
        end

        if(qr_flag == -1)
            continue;
        end



        %Possible qr code store middle position

        x1 = centerBlock(1) - round(centerBlock(4)/2);

        y1 = centerBlock(2);

        position = [x1; y1];
        qr_locations = [qr_locations, position];
        result_image(y1,x1) = 1.0;

    end
    
    if(verticalDirection)
        result_image = result_image'; % Transpose the image if we want to check vertically, instead of rewriting the loop for that case
        qr_locations = swap_rows(qr_locations, 1,2);
    end



end

