-------------------------------------------------------------------------------
--     ____  _____          __    __    ________    _______
--    |    | \    \        |   \ |  |  |__    __|  |   __  \
--    |____|  \____\       |    \|  |     |  |     |  |__>  ) 
--     ____   ____         |  |\ \  |     |  |     |   __  <
--    |    | |    |        |  | \   |     |  |     |  |__>  )
--    |____| |____|        |__|  \__|     |__|     |_______/
--
--    NTB University of Applied Sciences in Technology
--
--    Campus Buchs - Werdenbergstrasse 4 - 9471 Buchs - Switzerland
--    Campus Waldau - Schoenauweg 4 - 9013 St. Gallen - Switzerland
--
--    Web http://www.ntb.ch        Tel. +41 81 755 33 11
--
-------------------------------------------------------------------------------
-- Copyright 2013 NTB University of Applied Sciences in Technology
-------------------------------------------------------------------------------
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
-- 
-- http://www.apache.org/licenses/LICENSE-2.0
--   
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
-------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE IEEE.math_real.ALL;
use work.spi_cs_mux_pkg.all;

ENTITY spi_cs_mux_tb IS
END ENTITY spi_cs_mux_tb;

ARCHITECTURE sim OF spi_cs_mux_tb IS
	
	CONSTANT main_period : TIME := 10 ns; -- 100MHz
	CONSTANT spi_period : TIME := 125 ns; -- 8MHz

	
	SIGNAL sl_clk					: STD_LOGIC := '0';
	--processor side
	SIGNAL sl_selector				: STD_LOGIC := '1';
	SIGNAL isl_CS_n					: STD_LOGIC := '1';
	SIGNAL isl_SCLK					: STD_LOGIC := '1';
	SIGNAL isl_MOSI					: STD_LOGIC := '1';
	SIGNAL osl_MISO					: STD_LOGIC;
	
	--fpga side
	SIGNAL osl_CS_0_n				: STD_LOGIC;
	SIGNAL osl_CS_1_n				: STD_LOGIC;
	SIGNAL osl_SCLK					: STD_LOGIC;
	SIGNAL osl_MOSI					: STD_LOGIC;
	SIGNAL isl_MISO_MOT				: STD_LOGIC := '1';
	SIGNAL isl_MISO_SENS			: STD_LOGIC := '1';
	
	SIGNAL islv_ios					: STD_LOGIC_VECTOR(3 DOWNTO 0) := (OTHERS => '0');
	SIGNAL oslv_leds				: STD_LOGIC_VECTOR(3 DOWNTO 0);
	
	SIGNAL miso_data				: STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
	
	
	
BEGIN
	--create component
	my_unit_under_test : spi_cs_mux 
	PORT MAP(
			isl_clk						=> sl_clk,
			
			--processor side
			isl_selector			 	=> sl_selector,
			isl_CS_n					=> isl_CS_n,
			isl_SCLK					=> isl_SCLK,
			isl_MOSI					=> isl_MOSI,
			osl_MISO					=> osl_MISO,
			
			--fpga side
			osl_CS_0_n					=> osl_CS_0_n,
			osl_CS_1_n					=> osl_CS_1_n,
			osl_SCLK					=> osl_SCLK,
			osl_MOSI					=> osl_MOSI,
			isl_MISO_MOT				=> isl_MISO_MOT,
			isl_MISO_SENS				=> isl_MISO_SENS,
			
			-- led/ ios
			islv_ios					=> islv_ios,
			oslv_leds					=> oslv_leds
	);

	sl_clk 		<= NOT sl_clk after main_period/2;
	

	
	
	tb_main_proc : PROCESS
		
	
	PROCEDURE spi_transfer(dataIN: in STD_LOGIC_VECTOR(31 DOWNTO 0);dataOUT: in STD_LOGIC_VECTOR(31 DOWNTO 0)) IS
		BEGIN
		isl_CS_n <= '0';
		WAIT FOR 2*spi_period;
		FOR i IN 31 DOWNTO 0 LOOP 
			isl_MOSI <= dataIN(i);
			isl_SCLK <= '0'; --falling edge
			isl_MISO_SENS <=dataOUT(i);
			WAIT FOR spi_period;
			isl_SCLK <= '1'; --rising edge
			WAIT FOR spi_period;
		END LOOP;
		WAIT FOR spi_period;
		isl_CS_n <= '1';
		isl_MOSI <= '1';
		isl_MISO_SENS  <= '1';
 	END PROCEDURE spi_transfer;
	
	BEGIN

		islv_ios <= "1001";
		WAIT FOR 100*main_period;
			sl_selector <= '1';
			spi_transfer(x"C00A0000",x"AAAAAAAA");
		WAIT FOR 100*main_period;
			spi_transfer(x"000A0000",x"AAAAAAAA");
			WAIT FOR 100*main_period;
			ASSERT false REPORT "End of simulation" SEVERITY FAILURE;
	END PROCESS tb_main_proc;

END ARCHITECTURE sim;

