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
-- Project : 	DE2-115 Board Support Package
-- Unit    : 	SPI.vhd
-- Author  : 	David Frommelt
-- Created : 	October 2012
-------------------------------------------------------------------------------
-- Copyright(C) 2011: Interstate University of Applied Sciences NTB, Buchs
-------------------------------------------------------------------------------

-- 12bit encoder with additional 16 bit pulse-width-period counter for advanced 
-- velocity calculating
-- input of encoder signals (std_logic) are synchronized
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;

--------------------------------------------------------------------------------------
-- PACKAGE DEFINITION
--------------------------------------------------------------------------------------
PACKAGE encoder_16bit_pkg IS
	
	COMPONENT encoder_16bit
		PORT (
			isl_clk:				IN std_logic;
			isl_enc_A: 			IN std_logic;
			isl_enc_B: 			IN std_logic;
			ou_pos:				OUT signed(15 downto 0);
			ou_velo:				OUT signed(15 downto 0)
		);
	END COMPONENT encoder_16bit;

END PACKAGE encoder_16bit_pkg;	

--------------------------------------------------------------------------------------
-- MAIN FILE
--------------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE work.encoder_16bit_pkg.ALL;

--------------------------------------------------------------------------------------
-- ENTITIY
--------------------------------------------------------------------------------------
ENTITY encoder_16bit IS
		PORT (
			isl_clk:				IN std_logic;
			isl_enc_A: 			IN std_logic;
			isl_enc_B:			IN std_logic;
			ou_pos:				OUT signed(15 downto 0):=x"0000";
			ou_velo:				OUT signed(15 downto 0):=x"7FFF"
		);
END ENTITY encoder_16bit;

--------------------------------------------------------------------------------------
-- ARCHITECTURE
--------------------------------------------------------------------------------------
ARCHITECTURE encoder_12bit_rtl OF encoder_16bit IS
	TYPE t_encoder_register IS RECORD
		sl_enc_a1 :	std_logic;
		sl_enc_a2 :	std_logic;
		sl_enc_a3 :	std_logic;
		sl_enc_a4 :	std_logic;
		sl_enc_b1 :	std_logic;
		sl_enc_b2 :	std_logic;
		sl_enc_b3 :	std_logic;
		sl_enc_b4 :	std_logic;
		
		u_pos :	signed(15 downto 0);
		u_velo :	signed(15 downto 0);
		u_clk_cnt : signed(15 downto 0);
		sl_dir_velo:	std_logic;
	END RECORD;
	
	SIGNAL r, r_next : t_encoder_register;
	
	BEGIN
	
		--------------------------------------------
		-- combinatorial process
		--------------------------------------------
		comb_process: PROCESS(r, isl_enc_A, isl_enc_B)
		
		VARIABLE v: t_encoder_register;
		
		BEGIN
			v:=r;
			
			-- buffer input to synchronize asynchronous encoder data
			--FIFO: jeder Wert wird ein Feld weitergegeben. Der neue Wert wird abgespeichert, der älteste fliegt raus
			v.sl_enc_a4 := r.sl_enc_a3;
			v.sl_enc_a3 := r.sl_enc_a2;
			v.sl_enc_a2 := r.sl_enc_a1;
			v.sl_enc_a1 := isl_enc_A;
			
			v.sl_enc_b4 := r.sl_enc_b3;
			v.sl_enc_b3 := r.sl_enc_b2;
			v.sl_enc_b2 := r.sl_enc_b1;
			v.sl_enc_b1 := isl_enc_B;
			
			-- encoder pos counter on first edge
			-- pulse-width-period-counter only on first edge
			IF (r.sl_enc_a3='1' AND r.sl_enc_a4='0' AND r.sl_enc_b3='0') AND r.u_clk_cnt > 3 THEN
				IF r.sl_dir_velo = '1' THEN
					v.u_velo := r.u_clk_cnt;
				ELSE
					v.u_velo := x"7FFF";
				END IF;
				v.sl_dir_velo := '1';
				v.u_clk_cnt :=x"0000";
			ELSIF (r.sl_enc_a3='0' AND r.sl_enc_a4='1' AND r.sl_enc_b3='0') THEN
				IF r.sl_dir_velo = '0' THEN
					v.u_velo := - r.u_clk_cnt;
				ELSE
					v.u_velo := x"7FFF";
				END IF;
				v.sl_dir_velo := '0';
				v.u_clk_cnt :=x"0000";
			ELSE
				IF v.u_clk_cnt < x"7FFF" THEN
					v.u_clk_cnt := v.u_clk_cnt+1;
				ELSE
					v.u_clk_cnt :=x"7FFF";
					v.u_velo := x"7FFF";
				END IF;
			END IF;
			
			-- count all encoder ticks for position
			IF r.sl_enc_a3='1' AND r.sl_enc_a4='0' THEN
				IF r.sl_enc_b3 = '0' THEN
					v.u_pos := r.u_pos + 1;
				ELSE
					v.u_pos := r.u_pos - 1;
				END IF;
			ELSIF v.sl_enc_a3='0' AND v.sl_enc_a4='1' THEN
				IF v.sl_enc_b3 = '1' THEN
					v.u_pos := r.u_pos + 1;
				ELSE
					v.u_pos := r.u_pos - 1;
				END IF;
			ELSIF v.sl_enc_b3='1' AND v.sl_enc_b4='0' THEN
				IF v.sl_enc_a3 = '1' THEN
					v.u_pos := r.u_pos + 1;
				ELSE
					v.u_pos := r.u_pos - 1;
				END IF;
			ELSIF v.sl_enc_b3='0' AND v.sl_enc_b4='1' THEN
				IF v.sl_enc_a3 = '0' THEN
					v.u_pos := r.u_pos + 1;
				ELSE
					v.u_pos := r.u_pos - 1;
				END IF;
			END IF;
			
			-- setting outputs
			r_next <=v;
			ou_pos <= r.u_pos;
			ou_velo <= r.u_velo;
			
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
END ARCHITECTURE encoder_12bit_rtl;