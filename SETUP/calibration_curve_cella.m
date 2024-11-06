clc, clear all

data = readmatrix("dataset.xlsx");

% COLONNA 1 = peso (g)
% COLONNA 2 = AnalogueValue-507 
% COLONNA 3 = AnalogueValue-507 (abs)
% COLONNA 4 = Forza Peso (N)
% COLONNA 5 = Fondo scala (forza/valore)
% COLONNA 5 = Fondo scala (valore/forza)

peso        = data(:, 1); 
valore_neg  = data(:, 2);
valore_abs  = data(:, 3); 
forza_peso  = data(:, 4);
fondo_scala = data(:, 5);

fig = figure(1);
clf
plot(valore_abs, forza_peso,  'LineWidth', 3, 'Color', 'r')
grid on, hold on
xlabel('Analogue Value - 507')
ylabel('Forza Peso [N]')
title('CURVA DI CALIBRAZIONE')
subtitle('Valore - Forza Peso')

%%% CURVA DI CALIBRAZIONE
x = 0 : round(mean(abs(fondo_scala(2:end))),2) : 250;
coeff = polyfit(valore_abs, forza_peso, 1);
curva_calibrazione = polyval(coeff, x); 

plot(x, curva_calibrazione, 'LineWidth', 1.5, 'Color', 'b', 'LineStyle', '--')
%y1 = coeff(1)*x + coeff(2);

text(5, 37.5, sprintf('   y = %0.2f x %0.2f', coeff(1), coeff(2)), FontSize=12, FontWeight='bold', ...
    HorizontalAlignment='left', VerticalAlignment='middle')

legend('DATI', 'CURVA CALIBRAZIONE', 'Location','southeast')

name_figure = 'curva-calibrazione.png'; 
% saveas(fig, name_figure);

%%% 

y = coeff(1)*x+coeff(2); 
