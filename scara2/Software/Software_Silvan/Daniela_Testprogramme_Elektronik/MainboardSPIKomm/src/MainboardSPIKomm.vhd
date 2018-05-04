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
-- Project : 	Mainboard SPI Communikation
-- Unit    : 	mainBoardSPIKomm.vhd
-- Author  : 	David Frommelt
-- Created : 	January 2013
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------


--------------------------------------------------------------------------------------
-- MAIN FILE
--------------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;

--------------------------------------------------------------------------------------
-- ENTITIY
--------------------------------------------------------------------------------------
ENTITY mainBoardSPIKomm IS
		PORT (
			isl_clk:				IN std_logic;
			
			-- main SPI as slave
			isl_main_spi_sclk:	IN std_logic;
			isl_main_spi_cs_n:	IN std_logic;
			isl_main_spi_mosi:	IN std_logic;
			osl_main_spi_miso:	OUT std_logic;
			
			-- servo SPI (and gyro)
			osl_servo_spi_sclk:	OUT std_logic;
			osl_servo_spi_mosi:	OUT std_logic;
			isl_servo_spi_miso:	IN std_logic;
			osl_servo_spi_cs_servo:	OUT std_logic;
			osl_servo_spi_cs_hall:	OUT std_logic;
			osl_servo_spi_cs_acc:	OUT std_logic;
			
			-- ADC SPI
			osl_adc1_spi_sclk:	OUT std_logic;
			osl_adc1_spi_cs_n:	OUT std_logic;
			osl_adc1_spi_mosi:	OUT std_logic;
			isl_adc1_spi_miso:	IN std_logic;
			osl_adc2_spi_sclk:	OUT std_logic;
			osl_adc2_spi_cs_n:	OUT std_logic;
			osl_adc2_spi_mosi:	OUT std_logic;
			isl_adc2_spi_miso:	IN std_logic;
			
			-- TOGGLE
			isl_toggle_in:		IN std_logic;
			osl_toggle_out:	OUT std_logic;
			
			-- HOMING
			islv3_homing:		IN std_logic_vector(2 downto 0);
			
			-- WIFI
			osl_wifi_reset:	OUT std_logic;
			osl_wifi_awake:	OUT std_logic;
			
			-- LASER
			isl_laser_output:	IN std_logic;
			
			-- JOYSTICK
			isl_joystick_c:	IN std_logic;
			isl_joystick_d:	IN std_logic;
			
			-- HMI
			islv4_hmi_sw:		IN std_logic_vector(3 downto 0);
			oslv4_hmi_led:		OUT std_logic_vector(3 downto 0)
			
		);
END ENTITY mainBoardSPIKomm;

--------------------------------------------------------------------------------------
-- ARCHITECTURE
--------------------------------------------------------------------------------------
ARCHITECTURE mainBoardSPIKomm_rtl OF mainBoardSPIKomm IS
	-- inputs of main spi
	TYPE t_spi_main_in IS RECORD
		sclk:		std_logic;
		cs_n:		std_logic;
		mosi:		std_logic;
	END RECORD;
	-- outputs of main spi
	TYPE t_spi_main_out IS RECORD
		miso:		std_logic;
	END RECORD;
	-- inputs of slave spi
	TYPE t_spi_slav_in IS RECORD
		servo_miso:		std_logic;
		adc1_miso:		std_logic;
		adc2_miso:		std_logic;
	END RECORD;
	-- outputs of slave spi
	TYPE t_spi_slav_out IS RECORD
		servo_sclk:		std_logic;
		servo_cs_n:		std_logic;
		hall_cs_n:		std_logic;
		acc_cs_n:		std_logic;
		servo_mosi:		std_logic;
		adc1_sclk:		std_logic;
		adc1_cs_n:		std_logic;
		adc1_mosi:		std_logic;
		adc2_sclk:		std_logic;
		adc2_cs_n:		std_logic;
		adc2_mosi:		std_logic;
	END RECORD;
	
	-- main registers
	TYPE t_init_state IS (INIT, RUN);
	TYPE t_main_register IS RECORD
		init_state:			t_init_state;
		clk_cnt:				unsigned(31 DOWNTO 0);
		led_cnt:				UNSIGNED(2 DOWNTO 0);
		led_state:			std_logic_vector(3 downto 0);
		spi_bit:				INTEGER RANGE 16 DOWNTO -1;
		spi_active:			std_logic;
		spi_addr:			UNSIGNED(2 downto 0);
		spi_in_value:		std_logic_vector(15 downto 0);
		spi_out_value:		std_logic_vector(15 downto 0);
		main_in:				t_spi_main_in;
		main_clk_last:		std_logic;
		main_out:			t_spi_main_out;
		slav_in:				t_spi_slav_in;
		slav_out:			t_spi_slav_out;
		digi_in:				std_logic_vector(12 downto 0);
		digi_out:			std_logic_vector(12 downto 0);
		doSlavOut:			std_logic;
		
		cntTimer:			unsigned(19 downto 0);
	END RECORD;
	
	-- constant definition 
	-- all spi communication are cpol = 1 and cpha = 1
	CONSTANT CPOL : STD_LOGIC := '1';
	CONSTANT CPHA : STD_LOGIC := '1';
	
	-- register definitions
	SIGNAL r, r_next : t_main_register;
	
	BEGIN
	
		--------------------------------------------
		-- combinatorial process
		--------------------------------------------
		comb_process: PROCESS(r, isl_main_spi_sclk, isl_main_spi_cs_n, isl_main_spi_mosi, isl_servo_spi_miso, isl_adc1_spi_miso, isl_adc2_spi_miso, isl_toggle_in, islv4_hmi_sw, islv3_homing, isl_laser_output)
		
		VARIABLE v: t_main_register;
		
		BEGIN
		
		
			-- load register to variable
			v:=r;
			--------------------------------------------
			-- init
			--------------------------------------------
			IF r.init_state = INIT THEN
				-- init variables for fast blinking hmi
				v.init_state := RUN;
				v.digi_out := b"0000010101010";
				v.spi_in_value := b"0000000010101010";
			END IF;
			
			--------------------------------------------
			-- checking the spi status
			--------------------------------------------
			IF r.cntTimer < x"FFFFF" THEN
				v.cntTimer := r.cntTimer + 1;
			ELSE
				v.digi_out := b"0000010101010";
				v.spi_in_value := b"0000000010101010";
			END IF;
			
			--------------------------------------------
			-- spi communication
			--------------------------------------------
			IF r.spi_active='1' THEN
				IF (r.main_in.sclk ='1' AND r.main_clk_last = '0') XOR ((CPOL='1') XOR (CPHA='1')) THEN
					-- rising edge
					IF r.spi_bit >-1 THEN
						-- save spi input to local register
						v.spi_in_value(v.spi_bit) := v.main_in.mosi;
						v.spi_bit := v.spi_bit - 1;
					END IF;
				ELSIF (r.main_in.sclk ='0' AND r.main_clk_last = '1') XOR ((CPOL='1') XOR (CPHA='1')) THEN
					-- falling edge
					IF r.spi_bit < 13 OR (r.spi_bit < 14 AND (r.spi_addr = b"101" OR r.spi_addr = b"100")) THEN
						v.doSlavOut := '1';
					END IF;
					IF r.spi_addr = b"000" THEN
						-- output the data in the register
						v.main_out.miso := r.spi_out_value(r.spi_bit);
					END IF;
				END IF;
				
				IF r.spi_addr /=b"000" THEN
					-- set output to the input from the slave (miso)
					CASE r.spi_addr IS
						WHEN b"000" =>
							-- none
						WHEN b"001" =>
							v.main_out.miso := r.slav_in.servo_miso;
						WHEN b"010" =>
							v.main_out.miso := r.slav_in.servo_miso;
						WHEN b"011" =>
							v.main_out.miso := r.slav_in.servo_miso;
						WHEN b"100" =>
							v.main_out.miso := r.slav_in.adc1_miso;
						WHEN b"101" =>
							v.main_out.miso := r.slav_in.adc1_miso;
						WHEN b"110" =>
							v.main_out.miso := r.slav_in.adc2_miso;
						WHEN b"111" =>
							v.main_out.miso := '0';
						WHEN OTHERS =>
							-- none
					END CASE;
				END IF;
				
					
				IF r.main_in.cs_n='1' THEN
					-- spi not active
					v.spi_active := '0';
					v.main_out.miso :='Z';
					v.doSlavOut := '0';
					IF r.spi_addr = 0 THEN
						v.digi_out := r.spi_in_value(12 DOWNTO 0);
					END IF;
				END IF;
				
			ELSE
				-- spi not active
				v.main_out.miso :='Z';
				v.spi_bit :=15;
				v.spi_active := '0';
				
				-- set all chip select to high
				v.slav_out.servo_cs_n := '1';
				v.slav_out.hall_cs_n := '1';
				v.slav_out.acc_cs_n := '1';
				v.slav_out.adc1_cs_n := '1';
				v.slav_out.adc2_cs_n := '1';
				
				-- define next spi address
				v.spi_addr := UNSIGNED(r.spi_in_value(15 DOWNTO 13));
				v.doSlavOut := '0';
				
				IF r.main_in.cs_n='0' THEN
					-- start spi communication
					v.spi_active := '1';
					v.spi_out_value := (others => '0');
					
					-- reset the error timer
					v.cntTimer := (others => '0');
					
					-- set spi-out-value and chose right chip select
					CASE r.spi_addr IS
					WHEN b"000" =>
						-- none
						v.spi_out_value(12 DOWNTO 0) := r.digi_in;
					WHEN b"001" =>
						v.slav_out.servo_cs_n := '0';
					WHEN b"010" =>
						v.slav_out.hall_cs_n := '0';
					WHEN b"011" =>
						v.slav_out.acc_cs_n := '0';
					WHEN b"100" =>
						v.slav_out.adc1_cs_n := '0';
						v.spi_out_value(15 DOWNTO 13) := b"000"; 
					WHEN b"101" =>
						v.slav_out.adc1_cs_n := '0';	
						v.spi_out_value(15 DOWNTO 13) := b"001"; 
					WHEN b"110" =>
						v.slav_out.adc2_cs_n := '0';	
					WHEN b"111" =>
						-- none
					WHEN OTHERS => 
						-- none
					END CASE;
				END IF;
			END IF;
			
			--------------------------------------------
			-- set slave sclk and mosi to input
			--------------------------------------------
			
			v.slav_out.servo_sclk 	:= r.main_in.sclk;
			v.slav_out.adc1_sclk 	:= r.main_in.sclk;
			v.slav_out.adc2_sclk 	:= r.main_in.sclk;
			
			v.slav_out.servo_mosi 	:= '0';
			v.slav_out.adc1_mosi 	:= '0';
			v.slav_out.adc2_mosi 	:= '0';
			
			IF r.doSlavOut='1' THEN
				v.slav_out.servo_mosi 	:= r.main_in.mosi;
				v.slav_out.adc1_mosi 	:= r.main_in.mosi;
				v.slav_out.adc2_mosi 	:= r.main_in.mosi;
			END IF;
			
			--------------------------------------------
			-- buffer once
			--------------------------------------------
			v.main_in.sclk 		:= isl_main_spi_sclk;
			v.main_in.cs_n 		:= isl_main_spi_cs_n;
			v.main_in.mosi 		:= isl_main_spi_mosi;
			
			v.slav_in.servo_miso	:= isl_servo_spi_miso;
			v.slav_in.adc1_miso	:=	isl_adc1_spi_miso;
			v.slav_in.adc2_miso	:= isl_adc2_spi_miso;
			
			--------------------------------------------
			-- set outputs
			--------------------------------------------
			--IF isl_main_spi_cs_n='0' THEN
			--	osl_main_spi_miso <= isl_main_spi_mosi;
			--ELSE
			--	osl_main_spi_miso <= 'Z';
			--END IF;
			
			osl_main_spi_miso		<= r.main_out.miso;
			
			osl_servo_spi_sclk 	<= r.slav_out.servo_sclk;
			osl_servo_spi_mosi	<= r.slav_out.servo_mosi;
			osl_servo_spi_cs_servo	<= r.slav_out.servo_cs_n;
			osl_servo_spi_cs_hall	<= r.slav_out.hall_cs_n;
			osl_servo_spi_cs_acc		<= r.slav_out.acc_cs_n;
			
			osl_adc1_spi_sclk		<= r.slav_out.adc1_sclk;
			osl_adc1_spi_mosi		<= r.slav_out.adc1_mosi;
			osl_adc1_spi_cs_n		<= r.slav_out.adc1_cs_n;
			
			osl_adc2_spi_sclk		<= r.slav_out.adc2_sclk;
			osl_adc2_spi_mosi		<= r.slav_out.adc2_mosi;
			osl_adc2_spi_cs_n		<= r.slav_out.adc2_cs_n;
			
			--------------------------------------------
			-- digital input/output
			--------------------------------------------
			IF r.clk_cnt < x"00BEBC20" THEN
				v.clk_cnt := r.clk_cnt + 1;
			ELSE
				v.clk_cnt := (others => '0');
				v.led_cnt := r.led_cnt + 1;
			END IF;
			for i in 0 to 3 loop
				v.led_state(i) := '0';
				IF r.digi_out(1+i*2) = '1' AND r.digi_out(2+i*2) = '1' THEN
					v.led_state(i) := '1';
				ELSIF r.digi_out(1+i*2) = '1' THEN
					v.led_state(i) := r.led_cnt(0);
				ELSIF r.digi_out(2+i*2) = '1' THEN
					v.led_state(i) := r.led_cnt(2);
				END IF;	
			end loop;
			oslv4_hmi_led <= r.led_state;
			
			osl_toggle_out 		<= r.digi_out(0);
			osl_wifi_reset			<= r.digi_out(9);
			osl_wifi_awake			<= r.digi_out(10);
			
			v.digi_in(0)			:= isl_toggle_in;
			v.digi_in(4 DOWNTO 1):= islv4_hmi_sw;
			v.digi_in(7 DOWNTO 5):= islv3_homing;
			v.digi_in(8) 			:= isl_laser_output;
			v.digi_in(9)			:= isl_joystick_c;
			v.digi_in(10)			:= isl_joystick_d;
			
			--------------------------------------------
			-- set last clk for edge detection
			--------------------------------------------
			v.main_clk_last := r.main_in.sclk;
			
			-- setting register to variable
			r_next <= v;
			
		END PROCESS comb_process;
		
		--------------------------------------------
		-- registry process
		--------------------------------------------
		reg_process: PROCESS (isl_clk)
		BEGIN
			IF rising_edge(isl_clk) THEN
				r<=r_next;
			END IF;
		END PROCESS reg_process;
END ARCHITECTURE mainBoardSPIKomm_rtl;