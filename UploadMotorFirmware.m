
%% Upload motor firmware to RUMBA ATmega2560

clc;

% Board settings
port = "COM5";
board = "arduino:avr:mega:cpu=atmega2560";

% Location of existing C++ firmware
sourceFile = fullfile(pwd, "HoldPlatform.cpp");

% Create Arduino sketch directory
sketchFolder = fullfile(pwd, "RumbaX");

if ~isfolder(sketchFolder)
    mkdir(sketchFolder);
end

% Copy C++ firmware into Arduino sketch format
sketchFile = fullfile(sketchFolder, "RumbaX.ino");
copyfile(sourceFile, sketchFile);

% Compile and upload firmware
disp("Compiling and uploading firmware...");

cmd = sprintf( ...
    'arduino-cli compile --fqbn %s --upload --port %s "%s"', ...
    board, port, sketchFolder);

[status, output] = system(cmd);
disp(output);

if status == 0
    disp("Firmware uploaded successfully!");
else
    error("Firmware upload failed.");
end
