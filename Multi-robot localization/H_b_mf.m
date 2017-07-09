function H_b_sym = H_b_mf(x1,x2,x4,x5,x7,x8)
%H_B_MF
%    H_B_SYM = H_B_MF(X1,X2,X4,X5,X7,X8)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    09-Jul-2017 19:30:09

t2 = x1.^2;
t3 = x2.^2;
t4 = x7.^2;
t5 = x8.^2;
t9 = x1.*x7.*2.0;
t10 = x2.*x8.*2.0;
t6 = t2+t3+t4+t5-t9-t10;
t7 = 1.0./t6;
t8 = x2-x8;
t11 = x1-x7;
t12 = t7.*t11;
t13 = t7.*t8;
t14 = x4.^2;
t15 = x5.^2;
t19 = x4.*x7.*2.0;
t20 = x5.*x8.*2.0;
t16 = t4+t5+t14+t15-t19-t20;
t17 = 1.0./t16;
t18 = x5-x8;
t21 = x4-x7;
t22 = t17.*t21;
t23 = t17.*t18;
t27 = x1.*x4.*2.0;
t28 = x2.*x5.*2.0;
t24 = t2+t3+t14+t15-t27-t28;
t25 = 1.0./t24;
t26 = x2-x5;
t29 = x1-x4;
t30 = t25.*t29;
t31 = t25.*t26;
H_b_sym = reshape([-t7.*t8,-t13,0.0,0.0,-t25.*t26,-t31,t12,t12,0.0,0.0,t30,t30,0.0,-1.0,0.0,0.0,0.0,-1.0,0.0,0.0,-t17.*t18,-t23,t31,t31,0.0,0.0,t22,t22,-t30,-t30,0.0,0.0,0.0,-1.0,-1.0,0.0,t13,t13,t23,t23,0.0,0.0,-t12,-t12,-t22,-t22,0.0,0.0,-1.0,0.0,-1.0,0.0,0.0,0.0],[6,9]);
