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
-- Unit    : 	pwm8_12bit.vhd
-- Author  : 	David Frommelt
-- Created : 	October 2012
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------

-- Package zur Erzeugung eines Dreieck-PWM Profils mit 12 Bit Auflösung
-- inklusive:  Start-Signal für synchronisierter ADC-Messung (10x schneller)
--					CNT-Signal 

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;

PACKAGE pwm6_pkg IS
	TYPE t_array6_s12bit IS ARRAY (5 DOWNTO 0) OF SIGNED(11 DOWNTO 0);   
	TYPE t_array6_u11bit IS ARRAY (5 DOWNTO 0) OF UNSIGNED(10 DOWNTO 0);           	
	TYPE t_array6_s11bit IS ARRAY (5 DOWNTO 0) OF SIGNED(10 DOWNTO 0);    	
	
	 COMPONENT pwm6_12bit
		PORT (
			isl_clk : 				IN STD_LOGIC;
			it_values: 				IN t_array6_s12bit;
			islv6_activate: 		IN STD_LOGIC_VECTOR(5 DOWNTO 0);
			oslv6_pwmA: 			OUT STD_LOGIC_VECTOR(5 DOWNTO 0):= (OTHERS => '0');
			oslv6_pwmB: 			OUT STD_LOGIC_VECTOR(5 DOWNTO 0):= (OTHERS => '0');
			oslv6_meanVal_st: 	OUT STD_LOGIC_VECTOR(5 DOWNTO 0)--;
			--osl_adc_start: 		OUT STD_LOGIC
		);
	END COMPONENT pwm6_12bit;
END PACKAGE pwm6_pkg;	

-----------------------------------------------------------------------------
-- ENTITIY
-----------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.pwm6_pkg.ALL;

ENTITY pwm6_12bit IS
	PORT (	
		isl_clk : 			IN STD_LOGIC;
		it_values: 			IN t_array6_s12bit;
		islv6_activate: 	IN STD_LOGIC_VECTOR(5 DOWNTO 0);
		oslv6_pwmA: 		OUT STD_LOGIC_VECTOR(5 DOWNTO 0):= (OTHERS => '0');
		oslv6_pwmB: 		OUT STD_LOGIC_VECTOR(5 DOWNTO 0):= (OTHERS => '0');
		oslv6_meanVal_st: OUT STD_LOGIC_VECTOR(5 DOWNTO 0)--;
		--osl_adc_start: 	OUT STD_LOGIC
	) ;
END ENTITY pwm6_12bit;


-----------------------------------------------------------------------------
-- ARCHITECTURE main
-----------------------------------------------------------------------------
ARCHITECTURE pwm6_12bit_rtl of pwm6_12bit IS

	CONSTANT counter_value_max : 	SIGNED(12 DOWNTO 0) := b"0111111111011";
	-- maximum value of synchronised counter for adc
	-- N = 2^11 - 4

	TYPE ta_sig13 IS ARRAY (5 DOWNTO 0) OF SIGNED(12 DOWNTO 0);
	CONSTANT casig13_cnt_decr	: ta_sig13 := (
		b"1100000000000",	
		b"1101100110011",	
		b"1111001100110",	
		b"0000110011001",	
		b"0010011001100",	
		b"0011111111111"
	);
	
	TYPE r_state IS (INIT, RUN);
	-- TYPE DEFINITION OF REGISTER
	TYPE t_pwm_registers IS RECORD
		state:				r_state;
		clk_cnt: 			SIGNED(12 DOWNTO 0);
		clk_cnt_adc: 		UNSIGNED(11 DOWNTO 0);
		idx_regel:			UNSIGNED(3 DOWNTO 0);
		meanval_start:		STD_LOGIC_VECTOR(5 DOWNTO 0);
	END RECORD;
	
	SIGNAL r, r_next : t_pwm_registers;

	BEGIN
	-----------------------------------------------------------------------------
	-- registry process
	-----------------------------------------------------------------------------
	comb_procss: PROCESS(r, it_values,islv6_activate)
	VARIABLE v : t_pwm_registers;
	VARIABLE cnt_I : ta_sig13;

	BEGIN
		---------------------------------------------------------
		-- register output
		---------------------------------------------------------
		v:= r;
		
		---------------------------------------------------------
		-- state machine with init and run
		---------------------------------------------------------
		CASE r.state IS
		
		WHEN INIT =>
			---------------------------------------------------------
			-- set all values to zero or minimal value
			---------------------------------------------------------
			v.clk_cnt := (OTHERS => '0');
			v.clk_cnt(12) := '1';
			v.clk_cnt_adc := (OTHERS => '0');
			v.idx_regel := x"1";
			
			FOR i IN 5 DOWNTO 0 LOOP
				oslv6_pwmA(i) <= '0';
				oslv6_pwmB(i) <= '0';
				cnt_I(i) := (OTHERS => '0');
			END LOOP;
			v.state := RUN;
			
		WHEN RUN =>
			---------------------------------------------------------
			-- calculate the output for all pwm values
			-- set pwm output if couter is higher or lower than prefered pwm value
			---------------------------------------------------------
			FOR i IN 5 DOWNTO 0 LOOP
				-- adding phase to 
				cnt_I(i) := r.clk_cnt - casig13_cnt_decr(i);
				
				IF cnt_I(i)(12) /= cnt_I(i)(11) THEN
					cnt_I(i) := NOT cnt_I(i);
				END IF;
				
				IF SIGNED(cnt_I(i)(11 DOWNTO 0)) <= it_values(i) AND islv6_activate(i) = '1' THEN
					oslv6_pwmA(i) <= '1';
				ELSE
					oslv6_pwmA(i) <= '0';
				END IF;
				
				IF SIGNED(cnt_I(i)(11 DOWNTO 0)) <= SIGNED(NOT it_values(i)) AND islv6_activate(i) = '1' THEN
					oslv6_pwmB(i) <= '1';
				ELSE
					oslv6_pwmB(i) <= '0';
				END IF;
			END LOOP;
			
			---------------------------------------------------------
			-- create start-signal for analog-digital convertion
			-- and cnt-index (10 times faster than pwm)
			---------------------------------------------------------
			IF r.clk_cnt >= counter_value_max THEN
				v.idx_regel := x"1";
			ELSE
				IF r.idx_regel < x"9" THEN
					v.idx_regel := r.idx_regel + 1;
				ELSE
					v.idx_regel := x"0";
				END IF;
			END IF;
			
			---------------------------------------------------------
			-- create index signal for motors 1 to 5
			---------------------------------------------------------
			IF r.idx_regel = x"1" THEN
				v.meanval_start(0) := '1';
			ELSE
				v.meanval_start(0) := '0';
			END IF;
			IF r.idx_regel = x"2" THEN
				v.meanval_start(1) := '1';
			ELSE
				v.meanval_start(1) := '0';
			END IF;
			IF r.idx_regel = x"3" THEN
				v.meanval_start(2) := '1';
			ELSE
				v.meanval_start(2) := '0';
			END IF;
			IF r.idx_regel = x"4" THEN
				v.meanval_start(3) := '1';
			ELSE
				v.meanval_start(3) := '0';
			END IF;
			IF r.idx_regel = x"5" THEN
				v.meanval_start(4) := '1';
			ELSE
				v.meanval_start(4) := '0';
			END IF;
			IF r.idx_regel = x"6" THEN
				v.meanval_start(5) := '1';
			ELSE
				v.meanval_start(5) := '0';
			END IF;
			
			v.clk_cnt := v.clk_cnt + 4;
			
		WHEN OTHERS =>
			v.state := INIT;
		END CASE;
		
		---------------------------------------------------------
		-- output
		---------------------------------------------------------
		--osl_adc_start <= r.adc_start;
		oslv6_meanVal_st <= r.meanval_start;
		
		---------------------------------------------------------
		-- register input
		---------------------------------------------------------
		r_next <= v;
		
	END PROCESS;

	-----------------------------------------------------------------------------
	-- registry process
	-----------------------------------------------------------------------------
	reg_process: PROCESS(isl_clk)
	BEGIN
		IF rising_edge(isl_clk) THEN
			r<=r_next;
		END IF;
	END PROCESS;
	
END ARCHITECTURE pwm6_12bit_rtl;
	
