function e = getErrorVec(R, p, goal)
    % Get the "distance" towards the goal
    % e = getErrorVec(R, p, goal)
    %   R is 3x3 rotation matrix at time t
    %   p is 3x1 position vector at time t in m

    e = R' * (goal - p);
end
