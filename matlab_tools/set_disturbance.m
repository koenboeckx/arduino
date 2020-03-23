function reset_system(arduino, disturbance)

write(arduino, uint8(252));
write(arduino, single(disturbance));
end