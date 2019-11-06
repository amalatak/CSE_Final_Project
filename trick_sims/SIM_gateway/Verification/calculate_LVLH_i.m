function LVLH_i = calculate_LVLH_i(position, velocity)

cross_xy = cross(position, velocity);

x_hat = -position/(sqrt(position(1)^2 + position(2)^2 + position(3)^2));
z_hat = -cross_xy/norm(cross_xy); 
y_hat = cross(z_hat, x_hat);

LVLH_i = [x_hat'; y_hat'; z_hat'];

end