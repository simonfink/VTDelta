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

PACKAGE spi_cs_mux_pkg IS
	COMPONENT spi_cs_mux IS
		PORT (
			--clk 
			isl_clk					:	IN 	STD_LOGIC;
			
			--processor side
			isl_selector			:	IN 	STD_LOGIC;
			isl_CS_n					:	IN 	STD_LOGIC;
			isl_SCLK					:	IN 	STD_LOGIC;
			isl_MOSI					:	IN 	STD_LOGIC;
			osl_MISO					:	OUT 	STD_LOGIC;
			
			--fpga side
			osl_CS_0_n				:	OUT	STD_LOGIC;
			osl_CS_1_n				:	OUT	STD_LOGIC;
			osl_SCLK					:	OUT	STD_LOGIC;
			osl_MOSI					:	OUT 	STD_LOGIC;
			isl_MISO_MOT			:	IN 	STD_LOGIC;
			isl_MISO_SENS			:	IN 	STD_LOGIC;
			
			-- led/ ios
			islv_ios					:	IN 	STD_LOGIC_VECTOR(3 DOWNTO 0);
			oslv_leds					:	OUT 	STD_LOGIC_VECTOR(3 DOWNTO 0)
	);
	END COMPONENT spi_cs_mux;
END PACKAGE;



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
use work.sync_pkg.all;

ENTITY spi_cs_mux IS
	PORT (
			--clk 
			isl_clk					:	IN 	STD_LOGIC;
			
			--signals from processor side
			isl_selector			:	IN 	STD_LOGIC;
			isl_CS_n					:	IN 	STD_LOGIC;
			isl_SCLK					:	IN 	STD_LOGIC;
			isl_MOSI					:	IN 	STD_LOGIC;
			osl_MISO					:	OUT 	STD_LOGIC;
			
			--fpga side
			osl_CS_0_n				:	OUT	STD_LOGIC;
			osl_CS_1_n				:	OUT	STD_LOGIC;
			osl_SCLK					:	OUT	STD_LOGIC;
			osl_MOSI					:	OUT 	STD_LOGIC;
			isl_MISO_MOT			:	IN 	STD_LOGIC;
			isl_MISO_SENS			:	IN 	STD_LOGIC;
			
			-- led/ ios
			islv_ios					:	IN 	STD_LOGIC_VECTOR(3 DOWNTO 0);
			oslv_leds					:	OUT 	STD_LOGIC_VECTOR(3 DOWNTO 0)
	);

END ENTITY spi_cs_mux;

ARCHITECTURE rtl OF spi_cs_mux IS
	SIGNAL rxBuff: STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');
	--SIGNAL txBuff: STD_LOGIC_VECTOR(31 DOWNTO 0);
	SIGNAL rxCount: UNSIGNED(5 DOWNTO 0) := to_unsigned(31,6);
	SIGNAL txCount: UNSIGNED(5 DOWNTO 0) := to_unsigned(0,6);
	SIGNAL oldSCLK: STD_LOGIC := '1';

	SIGNAL cs_sync						: 	STD_LOGIC;
	SIGNAL sck_sync					: 	STD_LOGIC;
	SIGNAL mosi_sync					: 	STD_LOGIC;
	SIGNAL miso_sens_sync					: 	STD_LOGIC;

	
	
BEGIN



sync_cs: 	sync PORT MAP ( isl_clk, isl_CS_n,   cs_sync   );
sync_sck:	sync PORT MAP ( isl_clk, isl_SCLK,  sck_sync  );
sync_mosi:	sync PORT MAP ( isl_clk, isl_MOSI, mosi_sync );
sync_miso_sens:	sync PORT MAP ( isl_clk, isl_MISO_SENS, miso_sens_sync );

osl_SCLK <= sck_sync;
osl_MOSI <= mosi_sync;


mux: PROCESS(isl_clk,isl_selector,cs_sync,sck_sync,isl_MISO_MOT,miso_sens_sync,mosi_sync,txCount,islv_ios)
BEGIN
	IF isl_selector = '1' THEN

		--Sensor
		osl_CS_0_n <= '1';
		osl_CS_1_n <= cs_sync;
		IF cs_sync = '0' THEN
			IF rising_edge(isl_clk) THEN
				IF oldSCLK /= sck_sync THEN --edge
					IF sck_sync = '1' THEN --rising edge
						rxBuff(to_integer(rxCount)) <= mosi_sync;
						IF rxCount = 0 THEN
							IF rxBuff(31 DOWNTO 30) = "01" THEN
								oslv_leds <= rxBuff(19 DOWNTO 16);
							END IF;
							rxCount <= to_unsigned(31,6);
						ELSE
							rxCount <= rxCount - 1;
						END IF;

					ELSE --falling edge
							
							
							txCount <= txCount + 1;
							
					END IF;
				END IF;
				oldSCLK <= sck_sync;
			END IF;
			IF txCount = 0 THEN
				osl_MISO <= islv_ios(3);
			ELSIF txCount = 1 THEN
				osl_MISO <= islv_ios(2);
			ELSIF txCount = 2 THEN
				osl_MISO <= islv_ios(1);
			ELSIF txCount = 3 THEN
				osl_MISO <= islv_ios(0);		
			ELSE
				osl_MISO <= miso_sens_sync;
			END IF;	
		ELSE -- cs is high
			rxCount <= to_unsigned(31,6);
			txCount <= to_unsigned(0,6);
			osl_MISO <= miso_sens_sync;
		END IF;
		
	ELSE
		--Motor bridge
		osl_CS_0_n <= cs_sync;
		osl_CS_1_n <= '1';
		osl_MISO <= isl_MISO_MOT;
		
	END IF;
END PROCESS mux;













END ARCHITECTURE rtl;