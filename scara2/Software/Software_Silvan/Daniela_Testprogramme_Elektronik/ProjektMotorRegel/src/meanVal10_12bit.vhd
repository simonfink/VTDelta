-----------------------------------------------------------------------------
--     ____  _____          __    __    ________    _______
--    |    | \    \        |   \ |  |  |__    __|  |   __  \
--    |____|  \____\       |    \|  |     |  |     |  |__>  ) 
--     ____   ____         |  |\ \  |     |  |     |   __  <
--    |    | |    |        |  | \   |     |  |     |  |__>  )
--    |____| |____|        |__|  \__|     |__|     |_______/
--
--    INTERSTATE UNIVERSITY OF AAPLIED SCIENCES OF TECHNOLOGY
--
--    Campus Buchs - Werdenbergstrasse 4 - CH-9471 Buchs
--    Campus Waldau - Schoenauweg4 - CH9013 St. Gallen
--
--    Tel. +41 (0)81 755 33 11   Fax +41 (0)81 756 54 34
--
-------------------------------------------------------------------------------
-- Project : 	MotorStromRegelung
-- Unit    : 	meanVal10_12bit.vhd
-- Author  : 	David Frommelt
-- Created : 	October 2012
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------

-- file for calculating the mean value from 10 values with reduction of the upper 
-- and the lower value

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;

PACKAGE meanVal10_12bit_pkg IS
	 COMPONENT meanVal10_12bit
		PORT (
			isl_clk : IN STD_LOGIC;
			iu12_val: IN UNSIGNED(11 DOWNTO 0);
			isl_start: 	IN STD_LOGIC;
			isl_write:	IN STD_LOGIC;
			ou12_val:	OUT UNSIGNED(11 DOWNTO 0);
			osl_ready:	OUT STD_LOGIC
		);
	END COMPONENT meanVal10_12bit;
END PACKAGE meanVal10_12bit_pkg;	

-------------------------------------------------------------------------------
-- main file
-------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.pwm6_pkg.ALL;

ENTITY meanVal10_12bit IS
	PORT (	
			isl_clk : 		IN STD_LOGIC;
			iu12_val: 		IN UNSIGNED(11 DOWNTO 0);
			isl_start: 		IN STD_LOGIC;
			isl_write:		IN STD_LOGIC;
			ou12_val:		OUT UNSIGNED(11 DOWNTO 0);
			osl_ready:		OUT STD_LOGIC
	) ;
END ENTITY meanVal10_12bit;


-----------------------------------------------------------------------------
ARCHITECTURE meanVal10_12bit_rtl of meanVal10_12bit IS
	
	TYPE t_meanVal_registers IS RECORD
		sum:	  			UNSIGNED(14 DOWNTO 0);
		minVal: 			UNSIGNED(11 DOWNTO 0);
		maxVal: 			UNSIGNED(11 DOWNTO 0);
		outVal: 			UNSIGNED(11 DOWNTO 0);
		write_prev: 	STD_LOGIC;
		ready:			STD_LOGIC;
		start_prev:		STD_LOGIC;
		idx:				UNSIGNED(3 DOWNTO 0);
	END RECORD;
	
	SIGNAL r, r_next : t_meanVal_registers;

	BEGIN
	
	--------------------------------------------
	-- combinatory process
	--------------------------------------------
	comb_procss: PROCESS(r, iu12_val, isl_start, isl_write)
	VARIABLE v : t_meanVal_registers;

	BEGIN
		v:= r;
		IF isl_start = '1' AND v.start_prev = '0' THEN
			-- reset index
			v.idx := x"0";
		END IF;
		IF	isl_write='1' AND v.write_prev = '0' THEN
			v.ready 	:= '0';
			IF v.idx = x"0" THEN
				-- reset values to zero and buffer input as maxVal
				v.minVal := iu12_val;
				v.maxVal := iu12_val;
				v.sum  := (others => '0');
				v.sum(2) := '1';
			ELSIF r.idx = x"1" THEN
				-- if input higher than maxval -> set as new maxval
				IF iu12_val<r.minVal  THEN
					v.minVal := iu12_val;
				ELSE
					v.maxVal := iu12_val;
				END IF;
			ELSIF r.idx < x"A" THEN
				-- automaticly add all values
				IF iu12_val <= r.minVal THEN
					-- bei neuem minWert, wird der bestehende minWert zur Summe addiert und der aktuelle Wert im minWert gespeichert.
					v.sum		:= r.sum + r.minVal;
					v.minVal := iu12_val;
				ELSIF iu12_val >= r.maxVal THEN
					-- bei neuem maxWert, wird der bestehende maxWert zur Summe addiert und der aktuelle Wert im maxWert gespeichert.
					v.sum 	:= r.sum + r.maxVal;
					v.maxVal := iu12_val;
				ELSE
					-- liegt der aktuelle Wert innerhalb von min und max wird er zur Summe addiert.
					v.sum		:= r.sum + iu12_val;
				END IF;
			END IF;
			IF r.idx = x"9" THEN
				-- set output
				-- summe wird dividiert durch 8
				v.outVal := v.sum(14 DOWNTO 3);
				v.ready 	:= '1';
			END IF;
			IF r.idx < x"F" THEN
				-- increment index without overflow
				v.idx := r.idx + 1;
			END IF;
		END IF;
		
		-- buffer inputs and set outputs
		v.write_prev :=isl_write;
		v.start_prev :=isl_start;
		ou12_val <= r.outVal;
		osl_ready <= r.ready;
		r_next <= v;
	END PROCESS;
	
	--------------------------------------------
	-- registry process
	--------------------------------------------
	reg_process: PROCESS(isl_clk)
	BEGIN
		IF rising_edge(isl_clk) THEN
			r<=r_next;
		END IF;
		
	END PROCESS;
	
END ARCHITECTURE meanVal10_12bit_rtl;
	
