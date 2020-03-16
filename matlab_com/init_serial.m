function arduino=init_serial(n_meas_per_period,n_period,port)
%init serial communication with arduino
%n_meas_per_period: the number of measurements returned by the arduino at each period (depending on your arduino code)
%n_period: the number of sampling periods returned in the response
%port: the com port to which the arduino is connected ('/dev/ttyS0', 'COM1', ...)
%if port=[], do not open connection (usefull to work with saved measures)
arduino.n_meas_per_period=n_meas_per_period;
arduino.n_period=n_period;
arduino.BaudRateTX=9600;
arduino.BaudRateRX=1000000;
if ~isempty(port)
  timeout=1000;
  timeout=120; % 2 minutes is probably enough
  disp(['init_serial: buffer size = ' num2str(n_period*n_meas_per_period*4)]);
  arduino.com=serial(port,'BaudRate',arduino.BaudRateTX,'InputBufferSize',n_period*n_meas_per_period*4,'Timeout',timeout);%4 bytes per float
  fopen(arduino.com);
  pause(2);% Must wait before sending data to avoid com errors
  if (strcmp( get(arduino.com, 'Status'), 'open')) 
    disp('init_serial: Serial port successfully opened ...');
  else
    disp('****** init_serial: Failed to open serial port ...');
    keyboard;
  end
end
