function filtered = funcButter(signal, order, fc, fs, type)

[b,a] = butter(order,fc/(fs/2), type);
filtered = filter(b, a, signal);

end

