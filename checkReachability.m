function reachable = checkReachability(Rt, pt, goal, k)
    e = getErrorVec(Rt, pt, goal);

    if ((sqrt(e(2)^2 + e(1)^2) - (1 / k))^2 + e(3)^2) >= (1 / (k^2))
        reachable = true;
    else
        reachable = false;
    end

end
