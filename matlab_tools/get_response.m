function Y = get_response(arduino, w, n_samples)
%GET_RESPONSE Get sequence of measurements from the controller
%   Reference value is set at w

mode = uint8(255);
write(arduino, mode);

write(arduino, single(w));
write(arduino, uint32(n_samples));

n = round(read(arduino, 1, 'double')) % how many items per sample

Y = zeros(n, n_samples);
for i = 1:n_samples
    for j = 1:n
        Y(j, i) = read(arduino, 1, 'double');
    end
end

