function F = createTransition(X)

F = [   X(2);
    -2*X(3)*X(4)*X(2) - X(4)^2*X(1);
    0;
    0];
