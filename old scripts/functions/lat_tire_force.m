function Fy = lat_tire_force(mu, Fz, C_alpha, alpha)

H = atan((3*mu*Fz)/C_alpha);

abs_alpha = abs(alpha);


if( abs_alpha < H )
    Fy = -C_alpha*alpha + (C_alpha^2*abs_alpha*alpha)/(3*mu*Fz) - (1/3)*((C_alpha * alpha)^3)/((3*mu*Fz)^2);
else
    Fy = -mu*Fz*sign(alpha);
end

end