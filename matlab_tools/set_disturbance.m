function set_disturbance(arduino, disturbance)
%SET_DISTURBANCE set disturbace (interpretaion is system dependend)

write(arduino, uint8(252));
write(arduino, single(disturbance));
end