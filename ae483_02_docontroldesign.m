function [K, x_e, u_e] = ae483_02_docontroldesign(h, x, u, h_num, params)

% Choose an equilibrium point. (The one here is just an example.) Note that
% parameter values are from the struct "params" that was created earlier.
x_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
u_e = [0; 0; 0; params.m * params.g];

% Make sure (x_e, u_e) is *actually* an equilibrium point, in other words
% that it satisfies
%
%   0 = h(x_e, u_e)
%
% If the state was x_e and the input were u_e, then xdot would be 0, so the
% state would not change - it would be at "equilibrium."
h_num(x_e, u_e)     % <--- look at ouput and check that all elements are 0

% Linearize about equilibrium point. The matrix A is computed as follows:
%
%   jacobian(h, x) finds the jacobian of h with respect to x
%   subs( ..., [x; u], [x_e; u_e]) plugs in the equilibrium point
%   double( ... ) converts the result from symbolic to numeric format
%
% The same process is used to compute B. The result is a state-space model
%
%   xcdot = A xc + B uc
%
% where
%
%   xc = x - x_e        <--- difference between state and equilibrium value
%   uc = u - u_e        <--- difference between input and equilibrium value
%
A = double( subs( jacobian(h, x), [x; u], [x_e; u_e] ) );
B = double( subs( jacobian(h, u), [x; u], [x_e; u_e] ) );

% Use LQR to design a linear state feedback controller of the form
%
%   uc = -K * xc
%
% This is a very common choice of controller. Note that "PD control" is
% just a special case of linear state feedback. In LQR, the matrix K is
% chosen to minimize a weighted sum of squared xc (errors) and square uc
% (inputs). The matrices Q and R - typically diagonal, and with *positive*
% numbers on the diagonal - are the weights. In particular, the cost is:
%
%   x' * Q * x + u' * R * u
%
% If you make Q(2, 2) bigger, then you are increasing the cost of the
% second element of the state - in this case, the error in y position.
% Since K is chosen to minimize cost, then if you implemented the resulting
% controller, you would expect to see the y position error decrease.
%
% If you make R(4, 4) bigger, then you are increasing the cost of the
% fourth element of the input - in this case, the total thrust from all the
% rotors. Since K is chosen to minimize cost, then you would expect to see
% the thrust decrease (and, most likely, a corresponding increase in the
% state errors, because you the controller is not applying as much input).
%
% And so forth.
%
% The weights chosen here are just examples. Typically one would start
% increasing or decreasing them by orders of magnitude. Also see "Bryson's
% rule" (google it) for some guidelines about good choices for weights.
%
Q = diag([0.2, 0.2, 2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]);
R = 10*diag([1,1,1,1]);
K = lqr(A, B, Q, R);
mat2str(A)
mat2str(B)


end

