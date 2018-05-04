-------------------------------------------------------------------------------
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
-- Unit    : 	MotorStromRegelung.vhd
-- Author  : 	David Frommelt
-- Created : 	October 2012
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.pwm6_pkg.ALL;
USE work.spi_16bit_pkg.ALL;
USE work.encoder_16bit_pkg.ALL;

--------------------------------------------------------------------------------------
-- MAIN ENTITIY (chip signals)
--------------------------------------------------------------------------------------
ENTITY MotorStromRegelung IS
	PORT (	
		------------------- Clock Input ----------------------------  
		isl_clock_40MHz	: IN  std_logic ;						-- 50 MHz
		------------------- Push Button and Swicht -----------------
		oslv2_red_led		: OUT std_logic_vector(1 DOWNTO 0):=b"00"; 	-- LED Red[1:0]
		------------------- MAIN SPI -----------------------------------
		ir_main_spi			: IN t_spi_mout; 
		or_main_spi			: OUT t_spi_min; 
		isl_toggle			: IN std_logic;
		------------------- CURRENT SPI -----------------------------------
		--ir_current_spi		: IN t_spi_min_6ch;
		--or_current_spi		: OUT t_spi_mout_inOnly;
		------------------- ENCODER -----------------------------------
		islv6_enc_A			: IN std_logic_vector(5 downto 0);
		islv6_enc_B			: IN std_logic_vector(5 downto 0);
		------------------- PWM-VALUES -----------------------------------
		oslv6_pwmA			: OUT std_logic_vector(5 DOWNTO 0);
		oslv6_pwmB			: OUT std_logic_vector(5 DOWNTO 0)
	) ;
END ENTITY MotorStromRegelung;
	


--------------------------------------------------------------------------------------
-- ARCHITECTURE
--------------------------------------------------------------------------------------
ARCHITECTURE top OF MotorStromRegelung IS
	
	SIGNAL motor_break : std_logic;
	
	CONSTANT cslv16_spi_out_val : std_logic_vector(15 downto 0):= x"1800";
	--CONSTANT cslv16_spi_out_val : std_logic_vector(15 downto 0):= x"3456";
	SIGNAL a6slv16_spi_in_val : t_spi_16bit_a6_inValue;
	
	-- type definitions
	TYPE t_array06_unsigned16 	IS ARRAY (5 downto 0) OF unsigned(15 downto 0);     
	TYPE t_array06_unsigned12 	IS ARRAY (5 downto 0) OF unsigned(11 downto 0);   
	TYPE t_array06_signed16 	IS ARRAY (5 downto 0) OF signed(15 downto 0);  	
	TYPE t_array06_signed14 	IS ARRAY (5 downto 0) OF signed(13 downto 0);  	
	TYPE t_array06_signed12 	IS ARRAY (5 downto 0) OF signed(11 downto 0); 
	
	-- encoder
	SIGNAL a6u16_enc_pos : 		t_array06_signed16;
	SIGNAL a6u16_enc_velo : 	t_array06_signed16;
	SIGNAL spiHelp_out : 		t_spi_mout;
	
	-- main spi
	SIGNAL int_spi_main_bit : 	INTEGER RANGE 15 downto -1;
	SIGNAL sl_spi_main_ready : std_logic;
	SIGNAL slv16_spi_in_main : std_logic_vector(15 downto 0);
	
	SIGNAL slv6_adc_start:		STD_LOGIC_VECTOR(5 DOWNTO 0);
	SIGNAL usig4_idx_regel : 	UNSIGNED(3 DOWNTO 0);

	
	-- Regelung
	SIGNAL slv6_goToCalc: std_logic_vector(5 downto 0);
	CONSTANT pwm_max_val : signed(11 downto 0) := x"6FF";
	CONSTANT pwm_min_val : signed(11 downto 0) := -x"6FF";
	CONSTANT volt_max_val : signed(11 downto 0) := x"6AA";
	CONSTANT volt_min_val : signed(11 downto 0) := -x"6AA";
	
	-- encoder
	COMPONENT pwm_pll
		PORT
		(
			inclk0		: IN STD_LOGIC  := '0';
			c0				: OUT STD_LOGIC;
			c1 			: OUT STD_LOGIC;
			c2 			: OUT STD_LOGIC
		);
	END COMPONENT;


	SIGNAL sl_clock_160MHZ : std_logic;
	SIGNAL sl_clock_10MHZ : std_logic;
	SIGNAL sl_clock_32MHZ : std_logic;
	
	-- main registers
	TYPE t_spi_state IS (WaitSPI, SendSPI);
	TYPE t_regel_state IS (CalcDiff, Multiply, Finalize, MeasureI);
	TYPE r_main_registers IS RECORD
		spi_state : 	t_spi_state;
		spi_addr	: 		INTEGER RANGE -1 to 15;
		spi_addr_autoinc1 : std_logic;
		spi_addr_autoinc2 : std_logic;
		spi_out_main: 	std_logic_vector(15 downto 0);
		regel_state:	t_regel_state;
		current_sp : 	t_array06_signed12;
		
		init_counter:	unsigned(11 downto 0);
		--current_offset_sum: t_array06_unsigned16;
		
		multAin : 		signed(13 downto 0);
		multBin : 		signed(11 downto 0);
		regel_x_prev: 	t_array06_signed14;
		regel_gain : 	t_array06_signed12;
		pwm_val: 		t_array6_s12bit;
		activate:		std_logic_vector(5 downto 0);
		volt_limit:		std_logic_vector(5 downto 0);
		
		
		idx_regel:		UNSIGNED(3 DOWNTO 0);
		goToCalc_prev:	STD_LOGIC_VECTOR(5 DOWNTO 0);
		goToCalc_ff:	STD_LOGIC_VECTOR(5 DOWNTO 0);
		
		led_cnt:			unsigned(24 downto 0);
		led_state:		std_logic;
		
		toggle_cnt:		unsigned(20 downto 0);
		toggle_reset:	std_logic;
		toggle_prev1:	std_logic;
		toggle_prev2:	std_logic;
		toggle_prev3:	std_logic;
			
	END RECORD;
	
	SIGNAL r, r_next : r_main_registers;
	
BEGIN
	
	--------------------------------------------
	-- combinatorial process
	--------------------------------------------
	comb_proc : PROCESS (r, isl_toggle, a6u16_enc_pos,	a6u16_enc_velo, sl_spi_main_ready, slv16_spi_in_main, slv6_goToCalc, int_spi_main_bit)
		VARIABLE v: r_main_registers;
		VARIABLE temp1: signed(13 downto 0);
		VARIABLE temp2: unsigned(13 downto 0);
		VARIABLE temp3: unsigned(13 downto 0);
		VARIABLE regel_x: signed(13 downto 0);
		VARIABLE pwm_new: signed(21 downto 0);
		VARIABLE i_idx_regel: INTEGER RANGE 5 DOWNTO 0;
	BEGIN
		v:= r;
		
		-- toggle ansteuerung
		IF r.toggle_prev2 = '1' AND r.toggle_prev3='0' THEN			v.toggle_cnt := (others => '0');
			IF r.toggle_cnt < x"FFFFF" THEN
				v.toggle_reset := '0';
			END IF;
		ELSIF r.toggle_cnt < x"FFFFF" THEN
			v.toggle_cnt := r.toggle_cnt + 1;
		ELSE
			v.toggle_reset := '1';
		END IF;
			
		v.toggle_prev3 := r.toggle_prev2;
		v.toggle_prev2 := r.toggle_prev1;
		v.toggle_prev1 := isl_toggle;
		
		
		-- led ansteuerung
		IF r.led_cnt < x"FFFFFF" THEN
			v.led_cnt := r.led_cnt + 1;
		ELSE
			v.led_cnt := (others => '0');
			v.led_state := NOT r.led_state;
		END IF;
		oslv2_red_led(0) <= v.led_state;
		oslv2_red_led(1) <= NOT r.toggle_reset;
		
		-- spi-sollwert einlesen und positions ausgabe
		CASE r.spi_state IS 
			WHEN WaitSPI =>
				IF sl_spi_main_ready='0' THEN
					v.spi_state := SendSPI;
					CASE r.spi_addr IS
						WHEN 0 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(0));
						WHEN 1 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(0));
						WHEN 2 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(1));
						WHEN 3 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(1));
						WHEN 4 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(2));
						WHEN 5 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(2));
						WHEN 6 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(3));
						WHEN 7 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(3));
						WHEN 8 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(4));
						WHEN 9 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(4));
						WHEN 10 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_pos(5));
						WHEN 11 => 
							v.spi_out_main := std_logic_vector(a6u16_enc_velo(5));
						WHEN OTHERS => 
							v.spi_out_main := x"0000";
						END CASE;
					
				END IF;
			WHEN SendSPI =>
				-- wait until communication is finished and then go to the wait state 
				IF int_spi_main_bit = 11 AND slv16_spi_in_main(12)='1' THEN
						v.spi_out_main(11 downto 6) := r.volt_limit;
						v.spi_out_main(5 downto 0) := r.activate;
				END IF;
				IF sl_spi_main_ready='1' THEN
					v.spi_state := WaitSPI;
					IF slv16_spi_in_main(12)='1' THEN
						v.spi_addr := to_integer(unsigned(slv16_spi_in_main(9 downto 6)));
						v.spi_addr_autoinc1 := slv16_spi_in_main(10);
						v.spi_addr_autoinc2 := slv16_spi_in_main(11);
						IF v.toggle_reset ='0' THEN
							v.activate := slv16_spi_in_main(5 downto 0);
						ELSE
							v.activate := (others => '0');
						END IF;
					ELSE
						CASE r.spi_addr IS
						WHEN 0 => 
							v.regel_gain(0) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 1 => 
							v.current_sp(0) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 2 => 
							v.regel_gain(1) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 3 => 
							v.current_sp(1) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 4 => 
							v.regel_gain(2) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 5 => 
							v.current_sp(2) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 6 => 
							v.regel_gain(3) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 7 => 
							v.current_sp(3) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 8 => 
							v.regel_gain(4) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 9 => 
							v.current_sp(4) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 10 => 
							v.regel_gain(5) := signed(slv16_spi_in_main(11 downto 0));
						WHEN 11 => 
							v.current_sp(5) := signed(slv16_spi_in_main(11 downto 0));
						WHEN OTHERS => 
							-- none
						END CASE;
						
						IF r.spi_addr_autoinc1='1' THEN
							v.spi_addr := r.spi_addr + 1;
						ELSIF r.spi_addr_autoinc2='1' THEN
							v.spi_addr := r.spi_addr + 2;
						END IF;
						
					END IF;
				END IF;
			WHEN OTHERS =>
				v.spi_addr := 0;
				v.spi_state := WaitSPI;
		END CASE;

		-----------------------------------------------
		-- Spannungswert
		-----------------------------------------------			
		
		IF v.current_sp(r.spi_addr) < pwm_min_val THEN
			v.pwm_val(r.spi_addr) := pwm_min_val;
		ELSIF v.current_sp(r.spi_addr) > pwm_max_val THEN
			v.pwm_val(r.spi_addr) := pwm_max_val;
		ELSE
			v.pwm_val(r.spi_addr) := signed(v.current_sp(r.spi_addr)(11 downto 0));
		END IF;

		FOR i IN 0 TO 5 LOOP
			IF r.toggle_reset='1' THEN
				v.activate(i) := '0';
			END IF;
		END LOOP;
		
		r_next<= v;
	
	END PROCESS comb_proc;
	
	--------------------------------------------
	-- registry process
	--------------------------------------------
	reg_proc : PROCESS (isl_clock_40MHZ)
	VARIABLE motor_break : std_logic;
	BEGIN
		IF rising_edge(isl_clock_40MHZ) THEN
		
			r <= r_next;
			
		END IF;
	END PROCESS reg_proc;
	
	--------------------------------------------------------------------------------------
	-- PLL
	--------------------------------------------------------------------------------------
	u_pwm_pll: pwm_pll PORT MAP(
		inclk0 					=> isl_clock_40MHz,
		c0							=> sl_clock_160MHZ,
		c1							=> sl_clock_10MHZ,
		c2							=> sl_clock_32MHZ
	);
	
	--------------------------------------------------------------------------------------
	-- SPI-Main
	--------------------------------------------------------------------------------------
	u_main_spi: spi_16bit_1ch_slave PORT MAP(
			clk_fast			=> sl_clock_160MHZ,
			spi_out			=> or_main_spi,
			spi_in			=> ir_main_spi,
			spi_out_value	=> r.spi_out_main,
			spi_in_value	=> slv16_spi_in_main,
			spi_bit        => int_spi_main_bit,
			spi_ready	   => sl_spi_main_ready,
			cpol				=> '1',
			cpha				=> '1'
	);
	
	--------------------------------------------------------------------------------------
	-- PWM (1x)
	--------------------------------------------------------------------------------------
	u_pwm8_12bit: pwm6_12bit PORT MAP(
		isl_clk					=> sl_clock_160MHZ,
		it_values				=> r.pwm_val,
		islv6_activate			=> r.activate,
		oslv6_pwmA				=> oslv6_pwmA,
		oslv6_pwmB				=> oslv6_pwmB,
		oslv6_meanVal_st		=> slv6_adc_start--,
		--osl_adc_start			=> sl_spi_I_start
	);
	
	--------------------------------------------------------------------------------------
	-- ENCODER (6x)
	--------------------------------------------------------------------------------------
	u_encoder_16bit_all: FOR i IN 0 TO 5 GENERATE
		u_encoder_16bit: encoder_16bit PORT MAP(
				isl_clk				=> sl_clock_160MHZ,
				isl_enc_A			=> islv6_enc_A(i),
				isl_enc_B			=> islv6_enc_B(i),
				ou_pos				=> a6u16_enc_pos(i),
				ou_velo				=> a6u16_enc_velo(i)
		);
	end generate;

END ARCHITECTURE top;

	