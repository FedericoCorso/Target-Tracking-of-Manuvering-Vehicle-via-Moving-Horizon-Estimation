function z_k_plus_1 = CTRV_exact_DT(z_k,d_k,Ts)
    z_k_plus_1(1,1) = z_k(1,1) + 2*(z_k(3,1)/z_k(5,1))*sin(Ts*z_k(5,1)/2)*cos(z_k(4,1)+z_k(5,1)*Ts/2) + (Ts^2/2)*cos(z_k(4,1))*d_k(1,1);
    z_k_plus_1(2,1) = z_k(2,1) + 2*(z_k(3,1)/z_k(5,1))*sin(Ts*z_k(5,1)/2)*sin(z_k(4,1)+z_k(5,1)*Ts/2) + (Ts^2/2)*sin(z_k(4,1))*d_k(1,1);
    z_k_plus_1(3,1) = z_k(3,1) + Ts*d_k(1,1);
    z_k_plus_1(4,1) = z_k(4,1) + Ts*z_k(5,1)+(Ts^2/2)*d_k(2,1);
    z_k_plus_1(5,1) = z_k(5,1) + Ts*d_k(2,1); 
end