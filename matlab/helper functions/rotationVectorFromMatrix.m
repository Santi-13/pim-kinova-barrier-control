
function rvec = rotationVectorFromMatrix(T)
    % Extract the rotation part (R) from the homogeneous transformation matrix (T)
    R = T(1:3, 1:3);
    
    % Calculate the rotation vector
    theta = acos((trace(R) - 1) / 2);
    rvec = (theta / (2 * sin(theta))) * [R(3, 2) - R(2, 3);
                                         R(1, 3) - R(3, 1);
                                         R(2, 1) - R(1, 2)];
end