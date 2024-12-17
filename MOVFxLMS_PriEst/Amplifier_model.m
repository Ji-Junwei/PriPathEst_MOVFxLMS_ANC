function y_a = Amplifier_model(Vthr,y)
    if y <-Vthr 
        y_a = -Vthr;
    elseif y > Vthr 
        y_a = Vthr ;
    else
        y_a = y    ;
    end
end