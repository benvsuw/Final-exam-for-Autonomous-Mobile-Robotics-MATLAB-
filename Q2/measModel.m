function quadBack = measModel(quad, markers)
    Re = [0.1 0 ; 0 0.1];
    [ReV, Rev] = eig(Re); % Get eigenvectors/values
    e = ReV*sqrt(Rev)*randn(length(quad(1,:)),1);
    quadBack = quad + e';
end