function H_b_sym = H_b_mf(x1,x2,x4,x5)
%H_B_MF
%    H_B_SYM = H_B_MF(X1,X2,X4,X5)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    24-Aug-2017 16:18:10

t2 = x1.^2;
t3 = x2.^2;
t4 = x4.^2;
t5 = x5.^2;
t9 = x1.*x4.*2.0;
t10 = x2.*x5.*2.0;
t6 = t2+t3+t4+t5-t9-t10;
t7 = 1.0./t6;
t8 = x2-x5;
t11 = x1-x4;
t12 = t7.*t11;
t13 = t7.*t8;
H_b_sym = reshape([-t7.*t8,-t13,t12,t12,0.0,-1.0,t13,t13,-t12,-t12,-1.0,0.0],[2,6]);
