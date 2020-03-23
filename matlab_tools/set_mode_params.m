function set_mode_params(arduino, mode, w, params)
%SET_MODE_PARAMS set mode and params

write(arduino, uint8(mode));
write(arduino, single(w));
if length(params) > 0
    write(arduino, uint8(length(params)));
    for i = 1:length(params)
        write(arduino, single(params(i)));
    end
else
    write(arduino, uint8(0));
end
end