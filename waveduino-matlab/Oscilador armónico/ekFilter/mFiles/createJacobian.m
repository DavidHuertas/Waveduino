function A = createJacobian(X)


%Creates Jacobian Matrix
A = [   0           1               0               0;
    -X(4)^2     -2*X(3)*X(4)    -2*X(2)*X(4)    (-2*X(3)*X(2) - 2*X(4)*X(1));
    0           0               0               0;
    0           0               0               0];
