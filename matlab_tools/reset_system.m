function reset_system(arduino)
%RESET_SYSTEM reset system to initial state

write(arduino, uint8(253));
end