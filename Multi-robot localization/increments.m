function [DX, DY, DT] = increments(x_k1)

DX = []; DY = []; DT = [];
    temp_x = x_k1(1:3:end);
    temp_y = x_k1(2:3:end);
    temp_t = x_k1(3:3:end);
    for m=length(temp_x):-1:2
        for n=1:m-1
            DX = [DX, temp_x(m) - temp_x(n)];
            DY = [DY, temp_y(m) - temp_y(n)];
            DT = [DT, temp_t(m) - temp_t(n)];
        end
    end