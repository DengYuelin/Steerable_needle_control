function [sigma, sigma_sign] = getManifold(e)
    sigma = atan2(e(1), -e(2));
    sigma_sign = sign(sigma);
end
