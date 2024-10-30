function shifted = funcShift(shifted, new_in)

% Shift di 1 a sinistra (FIFO)
shifted(:, 1:end-1) = shifted(:, 2:end); 
shifted(:, end) = new_in;  

end

