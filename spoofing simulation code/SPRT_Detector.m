function [g_k, Alarm] = SPRT_Detector(test_statistic,Threshold, drift_term, difference)
g_k = test_statistic + difference - drift_term;
if g_k <= 0
    g_k = 0;
end

h = Threshold;

if g_k >= h
    Alarm = 1;
else
    Alarm = 0;
end

end