function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',10); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf([
        'a: Read current sensor (ADC counts)   b: Read current sonsor (mA)\n',...
        'c: Read encoder (counts)              d: Read encoder (deg)\n',...
        'e: Reset encoder                      f: Set PWM (-100 to 100)\n',...
        'g: Set current gains                  h: Get current gains\n',...
        'i: Set position gains                 j: Get position gains\n',...
        'k: Test current control               l: Go to angle (deg)\n',...
        'm: Load step trajectory               n: Load cubic trajectory\n',...
        'o: Execute trajectory                 p: Unpower the motor\n',...
        'q: Quit client                        r: Get mode\n']);
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'
            n = fscanf(mySerial,'%d');
            fprintf('ADC counts: %d\n',n);
        case 'b'
            n = fscanf(mySerial,'%f');
            fprintf('Current sensor(mA): %.3f\n',n);
        case 'c'
            n = fscanf(mySerial,'%d');
            fprintf('Encoder counts: %d\n',n);
        case 'd'
            n = fscanf(mySerial,'%f');
            fprintf('Encoder angle(deg): %.3f\n',n);
%             n = input('Enter number: '); % get the number to send
%             fprintf(mySerial, '%d\n',n); % send the number
%             n = fscanf(mySerial,'%d');   % get the incremented number back
%             fprintf('Read: %d\n',n);     % print it to the screen
        case 'e'
            fprintf('Encoder: reseted to 32768\n');
        case 'f'
            n = input('Enter PWM value(-100 to 100): ');
            fprintf(mySerial, '%d\n', n);  % send the number to PIC32
        case 'g'
            Kp = input('Enter current Kp_c: ');
            Ki = input('Enter current Ki_c: ');
            fprintf(mySerial, '%f %f\n',[Kp,Ki]);
        case 'h'
            n = fscanf(mySerial,'%f %f\n');
            fprintf('Kp_c = %.3f   Ki_c = %.3f\n', n);
        case 'i'
            Kp = input('Enter position Kp_p: ');
            Ki = input('Enter position Ki_p: ');
            Kd = input('Enter position Kd_p: ');
            fprintf(mySerial, '%f %f %f\n',[Kp,Ki,Kd]);
        case 'j'
            n = fscanf(mySerial,'%f %f %f\n');
            fprintf('Kp_p = %.3f  Ki_p = %.3f  Kd_p = %.3f\n', n);
        case 'k'
            figure
            read_plot_matrix(mySerial);
        case 'l'
            n = input('Enter desired angle: ');
            fprintf(mySerial, '%f\n', n);
        case 'm'
            n = input('Enter number of reference points: ');
            fprintf(mySerial, '%d\n', n);
            reflist = input('Input reference list: ');
            ref = genRef(reflist, 'step');
            for i = 1:length(ref)
                fprintf(mySerial, '%f\n', ref(i));
            end
        case 'n'
            n = input('Enter number of reference points: ');
            fprintf(mySerial, '%d\n', n);
            reflist = input('Input reference list: ');
            ref = genRef(reflist, 'cubic');
            for i = 1:length(ref)
                fprintf(mySerial, '%f\n', ref(i));
            end
        case 'o'
            figure
            read_plot_matrix_position(mySerial);
        case 'x'
            m = input('Enter 1st number: ');
            n = input('Enter 2nd number: ');
            fprintf(mySerial, '%d  %d\n',[m,n]);
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'p'
            fprintf('Motor has been unpowered\n');
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            n = fscanf(mySerial,'%d');
            switch n
                case 0
                    s = 'IDLE';
                case 1
                    s = 'PWM';
                case 2
                    s = 'ITEST';
                case 3
                    s = 'HOLD';
                case 4
                    s = 'TRACK';
            end
            fprintf('Mode: %s\n',s);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
