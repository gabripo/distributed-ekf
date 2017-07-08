function H_b_sym = H_b_mf(x1,x2,x4,x5,x7,x8)
%H_B_MF
%    H_B_SYM = H_B_MF(X1,X2,X4,X5,X7,X8)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Jul-2017 18:57:47

t2 = x4.^2;
t3 = x5.^2;
t4 = x7.^2;
t5 = x8.^2;
t9 = x4.*x7.*2.0;
t10 = x5.*x8.*2.0;
t6 = t2+t3+t4+t5-t9-t10;
t7 = 1.0./t6;
t8 = x5-x8;
t11 = x4-x7;
t12 = t7.*t11;
t13 = t7.*t8;
t14 = x1.^2;
t15 = x2.^2;
t19 = x1.*x4.*2.0;
t20 = x2.*x5.*2.0;
t16 = t2+t3+t14+t15-t19-t20;
t17 = 1.0./t16;
t18 = x2-x5;
t21 = x1-x4;
t22 = t17.*t21;
t23 = t17.*t18;
H_b_sym = reshape([0.0,0.0,-t17.*t18,-t23,0.0,0.0,t22,t22,0.0,-1.0,0.0,-1.0,-t7.*t8,-t13,t23,t23,t12,t12,-t22,-t22,0.0,0.0,-1.0,0.0,t13,t13,0.0,0.0,-t12,-t12,0.0,0.0,-1.0,0.0,0.0,0.0],[4,9]);
