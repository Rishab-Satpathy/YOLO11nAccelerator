# 125 MHz Core System Clock (8.0 ns period)
create_clock -period 8.000 -name clk -waveform {0.000 4.000} [get_ports clk]