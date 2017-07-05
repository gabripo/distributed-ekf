function increments = temp(x_tot, vecNUm)

% Combinations number (without repetitions)
comb = nchoosek(vecNUm, 2);

% Increments initialization
DX = []; DY = []; DT = [];

% Sub-vectors initialization extraction
x = x_tot(1:3:end);
y = x_tot(2:3:end);
t = x_tot(3:3:end);

for i=1:comb
    
   DX = [DX, x(i+1:end) - x(1:end-i)];
   DY = [DY, y(i+1:end) - y(1:end-i)];
   DT = [DT, t(i+1:end) - t(1:end-i)];
end
increments = [DX; DY; DT];