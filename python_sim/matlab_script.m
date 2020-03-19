t = tcpclient('localhost', 6001);

while 1
    gain = input('choose gain: ')
    data = uint8(gain);
    write(t, data);
end
clear t