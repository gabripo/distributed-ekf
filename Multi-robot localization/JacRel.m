function [H_d, H_o, H_b] = JacRel(x)
%% Function that creates the SYMBOLICAL jacobian of the relative measures w.r.t x

%%
% Symbolic x
x_sym = symbolizer(x, 'x');

% Simbolic increments creation
[DX, DY, DT] = increments(x_sym);

%% Symbolic relative distance
syms h_d
h_d = sqrt(DX.^2+DY.^2);

% Jacobian of h_d w.r.t x
H_d = simplify(jacobian(h_d, x_sym));

%% Symbolic relative orientation
syms h_o
h_o = DT;

% Jacobian of h_o w.r.t x
H_o = jacobian(h_o, x_sym);

%% Symbolic bearing angle
syms h_b
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
T = x_sym(length(x):-3:3);

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
simplify(h_b);

% Jacobian of h_b w.r.t x
H_b = simplify(jacobian(h_b, x_sym));
