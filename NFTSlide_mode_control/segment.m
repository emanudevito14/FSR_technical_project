function [p, dot_p, ddot_p, t] = segment(p0, pf, dot_p0, dot_pf, ddot_p0, ddot_pf, dddot_p0, dddot_pf, t0, tf, Ts)

% Input:
%   p0, pf             - [1x4] initial and final position/yaw vectors [x, y, z, psi]
%   dot_p0, dot_pf     - [1x4] initial and final velocity vectors
%   ddot_p0, ddot_pf   - [1x4] initial and final acceleration vectors
%   dddot_p0, dddot_pf - [1x4] initial and final jerk vectors
%   t0, tf             - Start and end time of the segment (seconds)
%   Ts                 - Sampling time
%
% Output:
%   p, dot_p, ddot_p   - (4 x N) matrices of sampled trajectories (position, velocity, acceleration)
%   t                  - (1 x N) vector of absolute timestamps [t0, tf]

    % Segment duration and exact number of sample points (including endpoints)
    T = tf - t0;
    N_steps = round(T / Ts) + 1;

    % Robust time vector generation using linspace
    t = linspace(t0, tf, N_steps);
    tau = t - t0; % Relative time vector tau from 0 to T

    % Constraint matrix (evaluated at tau = 0 and tau = T)
    A = [    0,      0,      0,     0,    0,   0, 0, 1;
           T^7,    T^6,    T^5,   T^4,  T^3, T^2, T, 1;
             0,      0,      0,     0,    0,   0, 1, 0;
         7*T^6,  6*T^5,  5*T^4, 4*T^3, 3*T^2, 2*T, 1, 0;
             0,      0,      0,     0,    0,   2, 0, 0;
        42*T^5, 30*T^4, 20*T^3, 12*T^2, 6*T,   2, 0, 0;
             0,      0,      0,     0,    6,   0, 0, 0;
       210*T^4, 120*T^3, 60*T^2, 24*T,   6,   0, 0, 0];

    N_dim = length(p0);
    p = zeros(N_dim, N_steps);
    dot_p = zeros(N_dim, N_steps);
    ddot_p = zeros(N_dim, N_steps);

    for j = 1:N_dim
        b = [p0(j); pf(j); dot_p0(j); dot_pf(j); ddot_p0(j); ddot_pf(j); dddot_p0(j); dddot_pf(j)];
        a = A \ b; % Coefficients [a7, a6, a5, a4, a3, a2, a1, a0]

        p(j, :)      = a(1)*tau.^7 + a(2)*tau.^6 + a(3)*tau.^5 + a(4)*tau.^4 + a(5)*tau.^3 + a(6)*tau.^2 + a(7)*tau + a(8);
        dot_p(j, :)  = 7*a(1)*tau.^6 + 6*a(2)*tau.^5 + 5*a(3)*tau.^4 + 4*a(4)*tau.^3 + 3*a(5)*tau.^2 + 2*a(6)*tau + a(7);
        ddot_p(j, :) = 42*a(1)*tau.^5 + 30*a(2)*tau.^4 + 20*a(3)*tau.^3 + 12*a(4)*tau.^2 + 6*a(5)*tau + a(6);
    end
end