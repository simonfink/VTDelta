transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work



vcom -93 -work work {../src/spi_cs_mux.vhd}
vcom -93 -work work {../sim/spi_cs_mux_tb.vhd}

vsim -t 1ps -L rtl_work -L work -voptargs="+acc" spi_cs_mux_tb 


if {[file exists wave.do]} {
	do wave.do 
}

view structure
view signals
run 150 ms
