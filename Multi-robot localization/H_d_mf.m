function H_d_sym = H_d_mf(x1,x2,x4,x5,x7,x8)
%H_D_MF
%    H_D_SYM = H_D_MF(X1,X2,X4,X5,X7,X8)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    23-Aug-2017 18:40:14

t2 = x1-x7;
t3 = x2-x8;
t4 = t2.^2;
t5 = t3.^2;
t6 = t4+t5;
t7 = 1.0./sqrt(t6);
t8 = t2.*t7;
t9 = t3.*t7;
t10 = x4-x7;
t11 = x5-x8;
t12 = t10.^2;
t13 = t11.^2;
t14 = t12+t13;
t15 = 1.0./sqrt(t14);
t16 = t10.*t15;
t17 = t11.*t15;
t18 = x1-x4;
t19 = x2-x5;
t20 = t18.^2;
t21 = t19.^2;
t22 = t20+t21;
t23 = 1.0./sqrt(t22);
t24 = t18.*t23;
t25 = t19.*t23;
H_d_sym = reshape([t8,0.0,t24,t9,0.0,t25,0.0,0.0,0.0,0.0,t16,-t24,0.0,t17,-t25,0.0,0.0,0.0,-t8,-t16,0.0,-t9,-t17,0.0,0.0,0.0,0.0],[3,9]);
