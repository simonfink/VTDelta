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
-- Unit    : 	SPI.vhd
-- Author  : 	David Frommelt
-- Created : 	October 2012
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------

-- spi-file for master and slave communication
-- extended for 8 miso-only slave adcs with 8 bits

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;

PACKAGE spi_16bit_pkg IS
	-- typedefinition for the outputs from the master
	TYPE t_spi_mout IS RECORD
		sclk : 		std_logic;
		cs_n :		std_logic;
		mosi :		std_logic;
	END RECORD;	
	
	-- typedefinition for the outputs from the master without mosi
	TYPE t_spi_mout_inOnly IS RECORD
		sclk : 		std_logic;
		cs_n :		std_logic;
	END RECORD;		
	
	-- typedefinition for the inputs of the master
	TYPE t_spi_min IS RECORD
		miso : 		std_logic;
	END RECORD;

	TYPE t_spi_min_6ch IS ARRAY (5 downto 0) OF t_spi_min;
	
	TYPE t_spi_16bit_a6_inValue IS ARRAY (5 downto 0) OF STD_LOGIC_VECTOR(15 downto 0);
	
	COMPONENT spi_16bit_1ch_master
		PORT (
			clk_double:		IN std_logic;
			spi_out: 		OUT t_spi_mout;
			spi_in:			IN t_spi_min;
			spi_out_value:	IN std_logic_vector(15 downto 0);
			spi_in_value:	OUT std_logic_vector(15 downto 0);
			spi_start:		IN std_logic;
			spi_ready:		OUT std_logic;
			spi_bit:			OUT INTEGER RANGE 15 downto 0;
			cpol:				IN std_logic;
			cpha:				IN std_logic
		);
	END COMPONENT spi_16bit_1ch_master;
	
	COMPONENT spi_16bit_6ch_master_inOnly
		PORT (
			clk_double:		IN std_logic;
			spi_out: 		OUT t_spi_mout_inOnly;
			spi_in:			IN t_spi_min_6ch;
			spi_in_value:	OUT t_spi_16bit_a6_inValue;
			spi_start:		IN std_logic;
			spi_ready:		OUT std_logic;
			spi_bit:			OUT INTEGER RANGE 15 downto 0;
			cpol:				IN std_logic;
			cpha:				IN std_logic
		);
	END COMPONENT spi_16bit_6ch_master_inOnly;
	
	COMPONENT spi_16bit_1ch_slave
		PORT (
			clk_fast:		IN std_logic;
			spi_out: 		OUT t_spi_min;
			spi_in:			IN t_spi_mout;
			spi_out_value:	IN std_logic_vector(15 downto 0);
			spi_in_value:	OUT std_logic_vector(15 downto 0);
			spi_ready:		OUT std_logic;
			spi_bit:			OUT INTEGER RANGE 15 downto 0;
			cpol:				IN std_logic;
			cpha:				IN std_logic
		);
	END COMPONENT spi_16bit_1ch_slave;
	
END PACKAGE spi_16bit_pkg;	


--------------------------------------------------------------------------------
-- MASTER, 1CH, in and out
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.spi_16bit_pkg.ALL;

ENTITY spi_16bit_1ch_master IS
	PORT (
		clk_double:		IN std_logic;
		spi_out: 		OUT t_spi_mout;
		spi_in:			IN t_spi_min;
		spi_out_value:	IN std_logic_vector(15 downto 0);
		spi_in_value:	OUT std_logic_vector(15 downto 0):=x"0000";
		spi_start:		IN std_logic;
		spi_ready:		OUT std_logic :='1';
		spi_bit:			OUT INTEGER RANGE 15 downto 0 := 15;
		cpol:				IN std_logic;
		cpha:				IN std_logic
	);
END ENTITY spi_16bit_1ch_master;

ARCHITECTURE spi_16bit_1ch_master_rtl OF spi_16bit_1ch_master IS
	TYPE t_spi_register IS RECORD
		spi_bit:			INTEGER RANGE -1 to 16;
		spi_clk:			std_logic;
		spi_active:		std_logic;	
		spi_out:			t_spi_mout;
		spi_in_value:	std_logic_vector(15 downto 0);
	END RECORD;
	
	SIGNAL r, r_next : t_spi_register;
	
	BEGIN
		--------------------------------------------
		-- combinatorial process
		--------------------------------------------
		comb_process: PROCESS(r, spi_start, spi_out_value, spi_in, cpol, cpha)
		
			VARIABLE v: t_spi_register;
			BEGIN
			v:=r;
			
			IF v.spi_active='1' THEN
				v.spi_out.sclk := v.spi_clk XNOR CPOL;
				
				IF r.spi_bit = -1 AND r.spi_clk = '1' THEN
					v.spi_active := '0';
				ELSIF (r.spi_clk XOR CPHA) = '1' THEN
					-- write
					v.spi_out.mosi := spi_out_value(v.spi_bit);
				ELSE
					-- read
					v.spi_in_value(r.spi_bit) := spi_in.miso;
					
					if v.spi_bit > -1 THEN
						v.spi_bit := r.spi_bit-1;
					END IF;
					
				END IF;
				v.spi_clk := NOT r.spi_clk;
				
			ELSE
				v.spi_out.sclk := cpol;
				v.spi_out.cs_n :='1';
				v.spi_out.mosi := spi_out_value(15);
				v.spi_bit := 15;
				v.spi_clk := '0';
				
				IF spi_start ='1' THEN
					v.spi_active := '1';
					v.spi_out.cs_n :='0';
				END IF;
			END IF;
			
			r_next <=v;
			
		END PROCESS comb_process;
		
		--------------------------------------------
		-- registry process
		--------------------------------------------
		reg_process: PROCESS (clk_double)
		BEGIN
			IF rising_edge(clk_double) THEN
				r<=r_next;
				spi_in_value <= r_next.spi_in_value;
				spi_out <= r_next.spi_out;
				spi_ready <= NOT r_next.spi_active;
				spi_bit <= r_next.spi_bit;
			END IF;
		END PROCESS reg_process;
END ARCHITECTURE spi_16bit_1ch_master_rtl;

--------------------------------------------------------------------------------
-- MASTER, 8CH, only input
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.spi_16bit_pkg.ALL;

ENTITY spi_16bit_6ch_master_inOnly IS
	PORT (
			clk_double:		IN std_logic;
			spi_out: 		OUT t_spi_mout_inOnly;
			spi_in:			IN t_spi_min_6ch;
			spi_in_value:	OUT t_spi_16bit_a6_inValue;
			spi_start:		IN std_logic;
			spi_ready:		OUT std_logic;
			spi_bit:			OUT INTEGER RANGE 15 downto 0;
			cpol:				IN std_logic;
			cpha:				IN std_logic
	);
END ENTITY spi_16bit_6ch_master_inOnly;

ARCHITECTURE spi_16bit_6ch_master_inOnly_rtl OF spi_16bit_6ch_master_inOnly IS
	TYPE t_spi_register IS RECORD
		spi_bit:			INTEGER RANGE -1 to 16;
		spi_clk:			std_logic;
		spi_active:		std_logic;	
		spi_out:			t_spi_mout_inOnly;
		spi_in_value:	t_spi_16bit_a6_inValue;
	END RECORD;
	
	SIGNAL r, r_next : t_spi_register;
	
	BEGIN
		--------------------------------------------
		-- combinatorial process
		--------------------------------------------
		comb_process: PROCESS(r, spi_start, spi_in, cpol, cpha)
		
			VARIABLE v: t_spi_register;
			BEGIN
			v:=r;
			
			IF v.spi_active='1' THEN
				v.spi_out.sclk := v.spi_clk XOR CPOL;
				v.spi_out.cs_n := '0';
				
				IF v.spi_bit = -1 AND v.spi_clk = '0' THEN
					v.spi_active := '0';
				ELSIF (v.spi_clk XOR CPHA) = '0' THEN
					-- write
					-- NOTHING TO WRITE
				ELSE
					-- read
					for i in 5 downto 0 loop
						v.spi_in_value(i)(v.spi_bit) := spi_in(i).miso;
					end loop;
					
					if v.spi_bit > -1 THEN
						v.spi_bit := v.spi_bit-1;
					END IF;
					
				END IF;
				v.spi_clk := NOT v.spi_clk;
				
			ELSE
				v.spi_out.sclk := cpol;
				v.spi_out.cs_n :='1';
				v.spi_bit := 15;
				v.spi_clk := '0';
				
				IF spi_start ='1' THEN
					v.spi_active := '1';
				END IF;
			END IF;
			
			spi_bit <= r.spi_bit;
			r_next <=v;
			
		END PROCESS comb_process;
		
		--------------------------------------------
		-- registry process
		--------------------------------------------
		reg_process: PROCESS (clk_double)
		BEGIN
			IF rising_edge(clk_double) THEN
				r<=r_next;
				spi_in_value <= r_next.spi_in_value;
				spi_out <= r_next.spi_out;
				spi_ready <= NOT r_next.spi_active;
			END IF;
		END PROCESS reg_process;
END ARCHITECTURE spi_16bit_6ch_master_inOnly_rtl;

--------------------------------------------------------------------------------
-- SLAVE
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.spi_16bit_pkg.ALL;

ENTITY spi_16bit_1ch_slave IS
	PORT (
		clk_fast:		IN std_logic;
		spi_out: 		OUT t_spi_min;
		spi_in:			IN t_spi_mout;
		spi_out_value:	IN std_logic_vector(15 downto 0);
		spi_in_value:	OUT std_logic_vector(15 downto 0);
		spi_ready:		OUT std_logic;
		spi_bit:			OUT INTEGER RANGE 15 downto 0;
		cpol:				IN std_logic;
		cpha:				IN std_logic
	);
END ENTITY spi_16bit_1ch_slave;

ARCHITECTURE spi_16bit_1ch_slave_rtl OF spi_16bit_1ch_slave IS
	TYPE t_spi_register IS RECORD
		spi_clk_last:	std_logic;
		spi_bit:			INTEGER RANGE -1 to 16;
		spi_active:		std_logic;	
		spi_out:			t_spi_min;
		spi_in:			t_spi_mout;
		spi_out_value:	std_logic_vector(15 downto 0);
		spi_in_value:	std_logic_vector(15 downto 0);
	END RECORD;
	
	SIGNAL r, r_next : t_spi_register;
	
	BEGIN
		--------------------------------------------
		-- combinatorial process
		--------------------------------------------
		comb_process: PROCESS(r, spi_out_value, spi_in, CPOL, CPHA)
		
			VARIABLE v: t_spi_register;
			BEGIN
			
			-- registry
			v:=r;
			
			-- input
			v.spi_in := spi_in;
			v.spi_out_value := spi_out_value;
			
			IF r.spi_active='1' THEN
				IF (r.spi_in.sclk ='1' AND v.spi_clk_last = '0') XOR ((CPOL='1') XOR (CPHA='1')) THEN
					-- rising edge
					IF r.spi_bit >-1 THEN
						v.spi_in_value(v.spi_bit) := v.spi_in.mosi;
						v.spi_bit := v.spi_bit - 1;
					END IF;
				ELSIF (r.spi_in.sclk ='0' AND r.spi_clk_last = '1') XOR ((CPOL='1') XOR (CPHA='1')) THEN
					-- falling edge
					v.spi_out.miso := spi_out_value(r.spi_bit);
				END IF;
				
				IF spi_in.cs_n='1' THEN
					v.spi_active := '0';
					v.spi_out.miso :='Z';
				END IF;
				
			ELSE
				v.spi_out.miso :='Z';
				v.spi_bit :=15;
				v.spi_active := '0';
				
				IF spi_in.cs_n='0' THEN
					v.spi_clk_last := spi_in.sclk;
					v.spi_active := '1';
					v.spi_out.miso := spi_out_value(v.spi_bit);
				END IF;
			END IF;
			
			
			v.spi_clk_last := r.spi_in.sclk;
			
			-- registry 
			r_next <=v;
			
			-- Output
			spi_out <= r.spi_out;
			spi_in_value <= r.spi_in_value;
			spi_bit <= r.spi_bit;
			spi_ready <= NOT r.spi_active;
		END PROCESS comb_process;
		
		--------------------------------------------
		-- registry process
		--------------------------------------------
		reg_process: PROCESS (clk_fast)
		BEGIN
			IF rising_edge(clk_fast) THEN
				r<=r_next;
				
			END IF;
		END PROCESS reg_process;
END ARCHITECTURE spi_16bit_1ch_slave_rtl;