function [dFK_com_f,dFK_com_s,dFK_com_b] = autogen_jacobians(h_f,l_f,l_s,m_b,m_f,m_s,r,theta_f,theta_m,theta_s,theta_dot_f,theta_dot_m,theta_dot_s,w_f)
%AUTOGEN_JACOBIANS
%    [DFK_COM_F,DFK_COM_S,DFK_COM_B] = AUTOGEN_JACOBIANS(H_F,L_F,L_S,M_B,M_F,M_S,R,THETA_F,THETA_M,THETA_S,THETA_DOT_F,THETA_DOT_M,THETA_DOT_S,W_F)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-May-2020 20:00:07

t2 = cos(theta_f);
t3 = sin(theta_f);
t4 = l_s.*m_s;
t5 = m_b+m_s;
t6 = r.*theta_m;
t7 = theta_f+theta_s;
t8 = m_f+t5;
t9 = cos(t7);
t10 = t2.*w_f;
t11 = h_f.*t3;
t12 = m_b.*t6;
t13 = l_f.*t3;
t14 = sin(t7);
t15 = -t10;
t16 = 1.0./t8;
t17 = t4+t12;
t18 = t5.*t16;
t20 = m_b.*r.*t14.*t16.*theta_dot_m;
t21 = t16.*t17;
dFK_com_f = [t20+theta_dot_f.*(t9.*t21+l_f.*t2.*t18)+t9.*t21.*theta_dot_s;theta_dot_f.*(t10-t11)];
if nargout > 1
    t19 = t18-1.0;
    t23 = -t21;
    t22 = l_f.*t2.*t19;
    t24 = l_s+t23;
    dFK_com_s = [t20+theta_dot_f.*(t22-t9.*t24)-t9.*t24.*theta_dot_s;-theta_dot_f.*(t11+t13+t15+l_s.*t14)-l_s.*t14.*theta_dot_s];
end
if nargout > 2
    t26 = t6+t23;
    dFK_com_b = [theta_dot_f.*(t22-t9.*t26)-t9.*t26.*theta_dot_s-t14.*theta_dot_m.*(r-m_b.*r.*t16);-theta_dot_f.*(t11+t13+t15+t6.*t14)+r.*t9.*theta_dot_m-t6.*t14.*theta_dot_s];
end