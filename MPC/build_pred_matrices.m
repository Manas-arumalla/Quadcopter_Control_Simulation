function [Phi, Gamma] = build_pred_matrices(N, dt)

A = [eye(3), dt*eye(3); zeros(3), eye(3)];
B = [0.5*dt^2*eye(3); dt*eye(3)];
nx = size(A,1); nu = size(B,2);

Phi = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);
for i = 1:N
    Phi((i-1)*nx + (1:nx), :) = A^i;
    for j = 1:i
        Gamma((i-1)*nx + (1:nx), (j-1)*nu + (1:nu)) = A^(i-j) * B;
    end
end
end
