%2D ODE Model for simple trajectory analysis, model is a 3DOF point mass glider

clear
clc
close all

function [value, isterminal, direction] = cruise_start(t, state)
    value = state(1)*sin(state(2));
    isterminal = 1;
    direction = -1;
end

function [value, isterminal, direction] = ground(t, state)
    value = state(3);
    isterminal = 1;
    direction = -1;
end

function [value, isterminal, direction] = dive(t, state)
    value = state(3) - 10000;
    isterminal = 1;
    direction = -1;
end

function [value, isterminal, direction] = dive_end(t, state)

    [~, a, ~, ~] = atmoscoesa(0);
    impact_speed = 3*a;

    value = [ ...
        state(1) - impact_speed;  % Event 1: Mach 3
        state(3)                  % Event 2: ground
    ];

    isterminal = [1; 1];

    direction = [ ...
         1;   % accelerating upward through Mach 3
        -1    % descending through h = 0
    ];

end
function [value, isterminal, direction] = hold_speed(t, state)
    [~, a, ~, ~] = atmoscoesa(0);
    impact_speed = 3*a;

    value = state(1) - impact_speed;
    isterminal = 1;
    direction = 1;
end

Mach_init = 8;
launch_angle = deg2rad(25);
init_height = 1;
[~, a, ~, ~] = atmoscoesa(init_height); %m/s
V0 = Mach_init*a;

state_0_climb= [V0; launch_angle; init_height; 0];
time_range_climb = [0 100];
options_climb = odeset('Events', @cruise_start);

[t_climb, state_climb, te_climb] = ...
    ode45(@ODE_climb, ...
          time_range_climb, ...
          state_0_climb, ...
          options_climb);

if isempty(te_climb)
    error('Climb never reached the apex before time limit.');
end

state_0_cruise = state_climb(end, :);
time_range_cruise = [t_climb(end), t_climb(end) + 240];
options_cruise = odeset('Events', @dive);

[t_cruise, state_cruise, te_cruise] = ...
    ode45(@ODE_cruise, ...
          time_range_cruise, ...
          state_0_cruise, ...
          options_cruise);

if isempty(te_cruise)
    error('Cruise never reached the 7000 m dive altitude.');
end


state_0_dive = state_cruise(end, :);

%point downwards to enter dive
state_0_dive(2) = deg2rad(-30);
time_range_dive = [t_cruise(end), t_cruise(end) + 240];
options_dive = odeset('Events', @dive_end);

[t_dive, state_dive, te_dive, ye_dive, ie_dive] = ode45(@ODE_dive, time_range_dive, state_0_dive, options_dive);


if ~isempty(ie_dive) && ie_dive(end) == 1

    % Mach 3 was reached
    state_0_hold = state_dive(end,:);

    time_range_hold = ...
        [t_dive(end), t_dive(end) + 240];

    options_hold = odeset('Events', @ground);

    [t_hold_speed, state_hold_speed] = ...
        ode45(@ODE_hold_velocity, ...
              time_range_hold, ...
              state_0_hold, ...
              options_hold);

else

    % Ground reached before Mach 3
    t_hold_speed = [];
    state_hold_speed = [];

end


if isempty(t_hold_speed)

    t_total = [ ...
        t_climb;
        t_cruise(2:end);
        t_dive(2:end)
    ];

    state_total = [ ...
        state_climb;
        state_cruise(2:end,:);
        state_dive(2:end,:)
    ];

else

    t_total = [ ...
        t_climb;
        t_cruise(2:end);
        t_dive(2:end);
        t_hold_speed(2:end)
    ];

    state_total = [ ...
        state_climb;
        state_cruise(2:end,:);
        state_dive(2:end,:);
        state_hold_speed(2:end,:)
    ];

end



%% VELOCITY

figure()
hold on

plot(t_total, state_total(:,1), 'LineWidth', 1, 'Color', 'Blue')

plot(t_climb(end), state_climb(end,1), ...
    'o', 'LineWidth', 2)

plot(t_cruise(end), state_cruise(end,1), ...
    'o', 'LineWidth', 2)

plot(t_dive(end), state_dive(end,1), ...
    'o', 'LineWidth', 2)
if ~isempty(t_hold_speed)
    plot(t_hold_speed(end), state_hold_speed(end,1), ...
        'o', 'LineWidth', 2)
end

xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('2D Trajectory Velocity')

legend('Velocity', ...
       'Climb End', ...
       'Cruise End', ...
       'Dive End', ...
       'Hold End')

hold off


%% FLIGHT PATH ANGLE

figure()
hold on

plot(t_total, rad2deg(state_total(:,2)), ...
    'LineWidth', 1, 'Color', 'Blue')

plot(t_climb(end), rad2deg(state_climb(end,2)), ...
    'o', 'LineWidth', 2)

plot(t_cruise(end), rad2deg(state_cruise(end,2)), ...
    'o', 'LineWidth', 2)

plot(t_dive(end), rad2deg(state_dive(end,2)), ...
    'o', 'LineWidth', 2)

if ~isempty(t_hold_speed)
    plot(t_hold_speed(end), state_hold_speed(end,1), ...
        'o', 'LineWidth', 2)
end

xlabel('Time (s)')
ylabel('Flight Path Angle (deg)')
title('2D Trajectory Flight Path Angle')

legend('Flight Path', ...
       'Climb End', ...
       'Cruise End', ...
       'Dive End', ...
       'Hold End')

hold off


%% HEIGHT

figure()
hold on

plot(t_total, state_total(:,3), ...
    'LineWidth', 1, 'Color', 'Blue')

plot(t_climb(end), state_climb(end,3), ...
    'o', 'LineWidth', 2)

plot(t_cruise(end), state_cruise(end,3), ...
    'o', 'LineWidth', 2)

plot(t_dive(end), state_dive(end,3), ...
    'o', 'LineWidth', 2)

if ~isempty(t_hold_speed)
    plot(t_hold_speed(end), state_hold_speed(end,1), ...
        'o', 'LineWidth', 2)
end

xlabel('Time (s)')
ylabel('Height (m)')
title('2D Trajectory Height')

legend('Height', ...
       'Climb End', ...
       'Cruise End', ...
       'Dive End', ...
       'Hold End')

hold off


%% HORIZONTAL POSITION

figure()
hold on

plot(t_total, state_total(:,4), ...
    'LineWidth', 1, 'Color', 'Blue')

plot(t_climb(end), state_climb(end,4), ...
    'o', 'LineWidth', 2)

plot(t_cruise(end), state_cruise(end,4), ...
    'o', 'LineWidth', 2)

plot(t_dive(end), state_dive(end,4), ...
    'o', 'LineWidth', 2)

if ~isempty(t_hold_speed)
    plot(t_hold_speed(end), state_hold_speed(end,1), ...
        'o', 'LineWidth', 2)
end

xlabel('Time (s)')
ylabel('Position (m)')
title('2D Trajectory Position')

legend('Vehicle Position', ...
       'Climb End', ...
       'Cruise End', ...
       'Dive End', ...
       'Hold End')

hold off


%% TRAJECTORY: HEIGHT VS POSITION

figure()
hold on

plot(state_total(:,4), state_total(:,3), ...
    'LineWidth', 1, 'Color', 'Blue')

plot(state_climb(end,4), state_climb(end,3), ...
    'o', 'LineWidth', 2)

plot(state_cruise(end,4), state_cruise(end,3), ...
    'o', 'LineWidth', 2)

plot(state_dive(end,4), state_dive(end,3), ...
    'o', 'LineWidth', 2)

if ~isempty(t_hold_speed)
    plot(t_hold_speed(end), state_hold_speed(end,1), ...
        'o', 'LineWidth', 2)
end

xlabel('Position (m)')
ylabel('Height (m)')
title('Trajectory Simulation')

legend('Trajectory', ...
       'Climb End', ...
       'Cruise End', ...
       'Dive End', ...
       'Hold End')

hold off
