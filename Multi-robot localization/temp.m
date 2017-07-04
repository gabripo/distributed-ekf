clc
clear all
n = 6;
x = rand(1, n);

comb = nchoosek(n, 2);


%     for j=1:n-1
%         for k=0:n-j
%             x(n-k) - x(n-k-j)
%         end
%     end
d = [];
for i=1:n
   d = [d, x(i+1:end) - x(1:end-i)];
    
end