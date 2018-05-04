onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_clk
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_selector
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_cs_n
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_sclk
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_mosi
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/osl_miso
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/osl_cs_0_n
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/osl_cs_1_n
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/osl_sclk
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/osl_mosi
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_miso_mot
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/isl_miso_sens
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/islv_ios
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/oslv_leds
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/rxbuff
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/rxcount
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/txcount
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/oldsclk
add wave -noupdate /spi_cs_mux_tb/my_unit_under_test/iosignal
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1000 ps} 0}
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {1000999050 ps} {1001000050 ps}
