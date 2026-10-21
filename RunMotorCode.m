
%% RUMBA Motor Test

% Connect to ATmega2560
if ~exist('s', 'var') || ~isa(s, 'serialport')
    s = serialport("COM5", 115200);
    pause(2);
    flush(s);
end

% ===== SELECT MOTOR =====
% 1 = X
% 2 = Y
% 3 = Z
% 4 = E0
% 5 = E1
% 6 = E2

MOTOR = 5;  % Motor selection

% Send motor command
write(s, uint8(num2str(MOTOR)), "uint8");

fprintf("Command sent to motor %d\n", MOTOR);
