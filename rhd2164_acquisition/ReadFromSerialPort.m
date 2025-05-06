function ReadFromSerialPort

% If received data looks corrupted, try to start running this function with
% the black RESET button on the NUCLEO board held down, then release RESET
% button.
SAMPLED_CHANNELS = 4;
TIMESTAMPS_TO_PLOT = 2500;
OFFLINE_TRANSFER = false;

% Bandpass Parameters
f_low = 1;
f_high = 10;

% Notch filter parameters
f0 = 60;             % Notch frequency (Hz)
Q = 35;              % Quality factor (higher = narrower notch)
bw = f0 / Q;         % Bandwidth
wo = f0 / (TIMESTAMPS_TO_PLOT/2);    % Normalized notch frequency (0–1)


% Connect to Serial Port that STM32H7 is on
% (may be different than COM3 on your system)
device = serialport("/dev/tty.usbserial-A50285BI", 3000000);
 

% Check for already present data on the port
preExistingBytes = device.NumBytesAvailable;
if preExistingBytes ~= 0
    read(device, preExistingBytes, "uint8");
end

total_samples = (TIMESTAMPS_TO_PLOT*SAMPLED_CHANNELS*2);
% (how 8 channels?), 1250 samples = 10,000
% 20,000 bytes (2 bytes per sample)

while 1
    if device.NumBytesAvailable >= total_samples*2

        signal = 0.195 * (read(device, total_samples, "uint16") - 32768);        
        
        % bandpass 1 to 100
        Wn = [f_low f_high] / (TIMESTAMPS_TO_PLOT/2);
        [b, a] = butter(2, Wn, 'bandpass');        
        signal = filtfilt(b, a, signal); 

        %Notch 60
        [b, a] = iirnotch(wo, bw / (TIMESTAMPS_TO_PLOT/2));
        readData = filtfilt(b, a, signal);  % Apply zero-phase notch filter

       
        for i=1:SAMPLED_CHANNELS*2
            subplot(SAMPLED_CHANNELS*2,1,i);
            plot(readData(i:SAMPLED_CHANNELS*2:end));

            ylim([-5000 5000]);


            title(['Received channel index: ', num2str(i)]);
        end
        drawnow;
        
        if OFFLINE_TRANSFER
            % Check for unexpected remaining bytes.
            remainderBytes = device.NumBytesAvailable;
            if (remainderBytes ~= 0)
                fprintf(1, 'ERROR Non-zero number of remaining bytes: %d\n', remainderBytes);
                remainderData = read(device, remainderBytes, "uint8");
            end
            delete(device);
            break
        end
    end
end
end