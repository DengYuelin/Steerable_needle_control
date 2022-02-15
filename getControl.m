function u = getControl(lambda1, lambda2, sigma_sign)
    u = [lambda1;
        lambda2 * sigma_sign];
end
