function [data] = ae483_03_simulate(h_num, K, x_e, u_e, z, x0xyz, x0ypr, num)

% Choose a sample rate. This is the number of times per second that the
% controller will run. This is also the number of times per second that
% data will be collected (at least, in the simulation I've created here -
% as you know, things are different in the hardware experiments, where the
% onboard controller and the offboard data logger run at different rates).
sampleRate = 50;

% Compute the time step. This is the length of time between one sample and
% the next.
tStep = 1 / sampleRate;

% Choose an integer number of time steps to run the simulation for (i.e.,
% samples to take). The total simulation time is then
%
%   num_samples / sample_rate
%
sampleNumber = 9000;

% Define the initial state (i.e., the state at time zero, at the very start
% of the simulation). To compare your results with an experiment, it would
% be very important to match the initial state here with the one you used
% in your experiment (so you should get it from your data).
x0 = [0; 0; 0; 0; 0; 0; 0; 0];
% x_e(3) = z;
% Define variables to keep track of the time, state, and input during the
% simulation. I like to put these variables all together in a struct - that
% makes it easy to store and pass the data around, and also to add more
% things to the data later on if I want (rotor spin rates, for example).
%
% I initialize the time and state with their initial values. I initialize
% the input as an empty matrix - no inputs have been chosen yet.
data = struct('t', 0, 'x', x0, 'u', []);

% Loop through all time steps.
for i = 1:sampleNumber
    
    % The "current time" is at time step "i". So, the current time and
    % state are in the i'th column of data.t and data.x.
    t = data.t(:, i);
    x = data.x(:, i);
    
    % Choose input based on the current time and state.
    % - Compute state of linear system.
    
    % x_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    if t > 5 && t < 10
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 10 && t < 15
        x_e = [1; 1; -1.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 15 && t < 20
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 20 && t < 25
        x_e = [1; -1; -1.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 25 && t < 30
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 30 && t < 35
        x_e = [1; 1; -0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 35 && t < 40
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 40 && t < 45
        x_e = [1; -1; -0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 45 && t < 50
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 50 && t < 55
        x_e = [-1; 1; -1.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 55 && t < 60
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 60 && t < 65
        x_e = [-1; -1; -1.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 65 && t < 70
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 70 && t < 75
        x_e = [-1; 1; -0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 75 && t < 80
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 80 && t < 85
        x_e = [-1; -1; -0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 85 && t < 90
        x_e = [0; 0; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    elseif t > 90
        x_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    end
    x_c = x - x_e;
    % - Compute input of linear system (by state feedback).
    u_c = -K * x_c;
    % u_c = -K * x_c + kRef*r;
    % - Compute input of original, nonlinear system.
    u = u_c + u_e;
    % - Store current input in data structure.
    data.u(:, i) = u;
    
    % Numerically integrate equations of motion to compute what the next
    % state will be given the current state and input.
    %
    %   @(t, x) h_num(x, u)     This is a anonymous function that ode45
    %                           will call to find xdot as a function of t
    %                           and x. The syntax means that when ode45
    %                           calls this anonymous function with (t, x),
    %                           this function will return the value
    %                           h_num(x, u). Note that the "x" comes from
    %                           ode45 and is changing all the time, while
    %                           the "u" is what we computed above, from the
    %                           controller, and is constant over each time
    %                           step. For help on anonymous functions, do:
    %
    %                               doc anonymous
    %
    %   [t, t + tStep]          This is the time interval to simulate. The
    %                           state (i.e., current) time is t, the final
    %                           (i.e., next) time is t + tStep.
    %
    %   x                       This is the state at the state of the time
    %                           interval that we want to simulate. In other
    %                           words, it is the current state.
    %
    [t_sol, x_sol] = ode45(@(t, x) h_num(x, u), [t, t + tStep], x);
    
    % Parse the solution that is returned by ode45 and store the next time
    % and state in our data structure. These will become the *current* time
    % and state the next time through our loop.
    %
    % Here is what ode45 returns:
    %
    %   t_sol       m x 1 matrix of times
    %   x_sol       m x n matrix of states, where the state time t_sol(i)
    %               is given by the 1 x n matrix x_sol(i, :)
    %
    % So, the "next time" is the last element of t_sol, and the "next
    % state" is the last row of x_sol. I always prefer to represent the
    % state as an n x 1 matrix instead of as a 1 x n matrix, so I take the
    % transpose of the result before storing it in data.
    data.t(:, i + 1) = t_sol(end, :);
    data.x(:, i + 1) = x_sol(end, :)';
end

end