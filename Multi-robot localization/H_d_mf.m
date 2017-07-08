function H_d_sym = H_d_mf(x1,x2,x4,x5)
%H_D_MF
%    H_D_SYM = H_D_MF(X1,X2,X4,X5)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Jul-2017 12:34:40

t2 = x1-x4;
t3 = x2-x5;
t4 = t2.^2;
t5 = t3.^2;
t6 = t4+t5;
t7 = 1.0./sqrt(t6);
t8 = x1.*2.0;
t9 = t8-x4.*2.0;
t10 = t7.*t9.*(1.0./2.0);
t11 = x2.*2.0;
t12 = t11-x5.*2.0;
t13 = t7.*t12.*(1.0./2.0);
H_d_sym = [t10,t13,0.0,-t10,-t13,0.0];