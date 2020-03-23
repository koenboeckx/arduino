function [Y, U] = get_response(arduino, w, n_samples)
%GET_RESPONSE Get sequence of measurements from the controller
%   Reference value is set at w

mode = uint8(255);
write(arduino, mode);

write(arduino, single(w));
write(arduino, uint8(n_samples));

Y = zeros(1, n_samples);
for i = 1:n_samples
	Y(i) = read(arduino, 1, 'double')
end


U = zeros(1, n_samples);
for i = 1:n_samples
	U(i) = read(arduino, 1, 'double')
end
