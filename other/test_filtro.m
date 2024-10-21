clc, clear all

fs = 100;
x = 0:1/fs:199;

y1 = randn(1,length(x));

figure(1), clf
subplot(3,1,1)
grid on, hold on
plot(x, y1, 'LineWidth',1)

step = 1;

if step == 0

    add_linear = zeros(size(x));
    
    r = 20; 
    add_linear(x<=r) = x(x<=r)*0.5;  
    add_linear(x>=r & x <=80) =  add_linear(x==r);
    
    % add_linear(add_linear<=0) = 0;

    subplot(3,1,1)
    grid on, hold on
    plot(x, add_linear,'LineWidth',1)
    y = y1+add_linear;

else
    add_steps = (floor(x / 10) * 3);

    y = y1+add_steps;

    subplot(3,1,1)
    grid on, hold on
    plot(x, add_steps,'LineWidth',1)

end

subplot(3,1,2)
grid on, hold on

plot(x,y, 'LineWidth',2)

fc = 0.6;
order = 4;
[b,a] = butter(order, fc/(fs/2),'high');
filtered = filtfilt(b,a,y);

subplot(3,1,2)
grid on, hold on
plot(x, filtered, 'LineWidth',1)
legend(["raw", "filtered"])

mean(y)
mean(filtered)


%%
subplot(3,1,3)
plot(x, cumsum(filtered),'LineWidth',1)
grid on 
