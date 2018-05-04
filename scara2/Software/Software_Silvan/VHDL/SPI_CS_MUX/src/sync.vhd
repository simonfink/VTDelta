library IEEE;
use IEEE.std_logic_1164.all;

package sync_pkg is

	component sync
		port
		(
			clk	: in	std_logic;
			d	: in	std_logic; 
			q	: out	std_logic
		);
	end component sync;
	
end package sync_pkg;

--------------------------------------------------------------------------------


library IEEE;
use IEEE.std_logic_1164.all;

entity sync is
	port
	(
		clk	: in	std_logic;
		d	: in	std_logic; 
		q	: out	std_logic
	);
end entity sync;

architecture rtl of sync is
begin

	S: process(clk)
	begin
		if (rising_edge(clk)) then
			q <= d;
		end if;
	end process S;

end architecture rtl;
