function h = evalRel(x, fail)

[DX, DY, DT] = increments(x);

h_d = sqrt(DX.^2+DY.^2);

h_o = DT;

h_b = [];
% Splitting increments vectors 
% Position intialization and number of vehicles 
a = 1; n = length(x)/3; j=1;
for i=2:n
    X{j} = DX(a:n-i+a);
    Y{j} = DY(a:n-i+a);
    a = (n-i+a)+1; j=j+1;
end
clear i j

% Symbolic vector of orientations - reverse order [T_n,...,T_1]
T = x(length(x):-3:3);

% Symbolic bearing angle
for i=1:length(Y)   % Reading cell arrays
    k=length(T);
    % Reading cell element containing the increments with respect to T(i)
    for j=1:length(Y{i})
        h_b = [h_b, bearing( X{i}(j), Y{i}(j), T(i) ),...
                    bearing( X{i}(j), Y{i}(j), T(k) )];
        k=k-1;  % Decrasing k to shift the orientation
    end
end
clear i j k

% Including GPS or not
if fail
    % GPS not working
    h = [h_b'; h_d'; h_o'];
else
    h = [x; h_b'; h_d'; h_o'];
end