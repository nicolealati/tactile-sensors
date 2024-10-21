clc, clear all

fs = 100;
x = 0:1/fs:199;
n = length(x);

y1 = randn(1,n);

figure(1), clf
subplot(3,1,1)
grid on, hold on
plot(x, y1, 'LineWidth',1)

segment_length = floor(n / 5);

r = 20; 

add_linear = zeros(1, n);

add_linear(1:segment_length) = 0;
add_linear(segment_length+1:2*segment_length) = r*linspace(0, 1, segment_length);
add_linear(2*segment_length+1:3*segment_length) = r;
add_linear(3*segment_length+1:4*segment_length) = r*linspace(1, 0, segment_length);
add_linear(4*segment_length+1:end) = 0;

subplot(3,1,1)
grid on, hold on
plot(x, add_linear,'LineWidth',1)
y = y1+add_linear;

ylim([-5, 25])

y_mean = mean(y); 

subplot(3,1,2)
grid on, hold on
plot(x, y,'LineWidth', 0.8)
plot(x, ones(1,n)*y_mean, '--', 'LineWidth',2)

legend("raw", "mean value")

y_detrended = detrend(y); 
detrended_mean = mean(y_detrended);

subplot(3,1,3)
grid on, hold on
plot(x, y_detrended,'LineWidth', 0.8)
plot(x, ones(1,n)*detrended_mean, '--', 'LineWidth',2)

legend("detrended", "mean value")






