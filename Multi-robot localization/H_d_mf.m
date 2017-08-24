function H_d_sym = H_d_mf(x1,x2,x4,x5)
%H_D_MF
%    H_D_SYM = H_D_MF(X1,X2,X4,X5)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    24-Aug-2017 16:43:19

t2 = x1-x4;
t3 = x2-x5;
t4 = t2.^2;
t5 = t3.^2;
t6 = t4+t5;
t7 = 1.0./sqrt(t6);
t8 = t2.*t7;
t9 = t3.*t7;
H_d_sym = [t8,t9,0.0,-t8,-t9,0.0];
