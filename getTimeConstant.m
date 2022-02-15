function tf = getTimeConstant(e0, u, q, k, t)

    ax = e0(2) * u(2) / q + k * u(1)^2 * u(2) / q^3;
    bx = e0(1) * u(2)^2 / q^2 - e0(3) * k * u(1) * u(2) / q^2;
    cx =- k * u(1)^2 * u(2) / q^2;
    dx = -bx + e0(1);

    f1 = sign(cx * t +dx - sqrt(ax^2 + bx^2));
    f2 = sign(cx * t +dx + sqrt(ax^2 + bx^2));

    if f1 ~= f2
        tf = true;
    else
        tf = false;
    end

end
