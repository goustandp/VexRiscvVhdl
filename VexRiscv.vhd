-- Generator : SpinalHDL v1.3.8    git head : 57d97088b91271a094cebad32ed86479199955df
-- Date      : 08/03/2020, 09:18:47
-- Component : VexRiscv

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.all;

package pkg_enum is
  type AluCtrlEnum is (ADD_SUB,SLT_SLTU,BITWISE);
  type AluBitwiseCtrlEnum is (XOR_1,OR_1,AND_1);
  type ShiftCtrlEnum is (DISABLE_1,SLL_1,SRL_1,SRA_1);
  type EnvCtrlEnum is (NONE,XRET);
  type BranchCtrlEnum is (INC,B,JAL,JALR);
  type Src2CtrlEnum is (RS,IMI,IMS,PC);
  type Src1CtrlEnum is (RS,IMU,PC_INCREMENT,URS1);

  function pkg_mux (sel : std_logic;one : AluCtrlEnum;zero : AluCtrlEnum) return AluCtrlEnum;
  subtype AluCtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant AluCtrlEnum_defaultEncoding_ADD_SUB : AluCtrlEnum_defaultEncoding_type := "00";
  constant AluCtrlEnum_defaultEncoding_SLT_SLTU : AluCtrlEnum_defaultEncoding_type := "01";
  constant AluCtrlEnum_defaultEncoding_BITWISE : AluCtrlEnum_defaultEncoding_type := "10";

  function pkg_mux (sel : std_logic;one : AluBitwiseCtrlEnum;zero : AluBitwiseCtrlEnum) return AluBitwiseCtrlEnum;
  subtype AluBitwiseCtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : AluBitwiseCtrlEnum_defaultEncoding_type := "00";
  constant AluBitwiseCtrlEnum_defaultEncoding_OR_1 : AluBitwiseCtrlEnum_defaultEncoding_type := "01";
  constant AluBitwiseCtrlEnum_defaultEncoding_AND_1 : AluBitwiseCtrlEnum_defaultEncoding_type := "10";

  function pkg_mux (sel : std_logic;one : ShiftCtrlEnum;zero : ShiftCtrlEnum) return ShiftCtrlEnum;
  subtype ShiftCtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant ShiftCtrlEnum_defaultEncoding_DISABLE_1 : ShiftCtrlEnum_defaultEncoding_type := "00";
  constant ShiftCtrlEnum_defaultEncoding_SLL_1 : ShiftCtrlEnum_defaultEncoding_type := "01";
  constant ShiftCtrlEnum_defaultEncoding_SRL_1 : ShiftCtrlEnum_defaultEncoding_type := "10";
  constant ShiftCtrlEnum_defaultEncoding_SRA_1 : ShiftCtrlEnum_defaultEncoding_type := "11";

  function pkg_mux (sel : std_logic;one : EnvCtrlEnum;zero : EnvCtrlEnum) return EnvCtrlEnum;
  subtype EnvCtrlEnum_defaultEncoding_type is std_logic_vector(0 downto 0);
  constant EnvCtrlEnum_defaultEncoding_NONE : EnvCtrlEnum_defaultEncoding_type := "0";
  constant EnvCtrlEnum_defaultEncoding_XRET : EnvCtrlEnum_defaultEncoding_type := "1";

  function pkg_mux (sel : std_logic;one : BranchCtrlEnum;zero : BranchCtrlEnum) return BranchCtrlEnum;
  subtype BranchCtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant BranchCtrlEnum_defaultEncoding_INC : BranchCtrlEnum_defaultEncoding_type := "00";
  constant BranchCtrlEnum_defaultEncoding_B : BranchCtrlEnum_defaultEncoding_type := "01";
  constant BranchCtrlEnum_defaultEncoding_JAL : BranchCtrlEnum_defaultEncoding_type := "10";
  constant BranchCtrlEnum_defaultEncoding_JALR : BranchCtrlEnum_defaultEncoding_type := "11";

  function pkg_mux (sel : std_logic;one : Src2CtrlEnum;zero : Src2CtrlEnum) return Src2CtrlEnum;
  subtype Src2CtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant Src2CtrlEnum_defaultEncoding_RS : Src2CtrlEnum_defaultEncoding_type := "00";
  constant Src2CtrlEnum_defaultEncoding_IMI : Src2CtrlEnum_defaultEncoding_type := "01";
  constant Src2CtrlEnum_defaultEncoding_IMS : Src2CtrlEnum_defaultEncoding_type := "10";
  constant Src2CtrlEnum_defaultEncoding_PC : Src2CtrlEnum_defaultEncoding_type := "11";

  function pkg_mux (sel : std_logic;one : Src1CtrlEnum;zero : Src1CtrlEnum) return Src1CtrlEnum;
  subtype Src1CtrlEnum_defaultEncoding_type is std_logic_vector(1 downto 0);
  constant Src1CtrlEnum_defaultEncoding_RS : Src1CtrlEnum_defaultEncoding_type := "00";
  constant Src1CtrlEnum_defaultEncoding_IMU : Src1CtrlEnum_defaultEncoding_type := "01";
  constant Src1CtrlEnum_defaultEncoding_PC_INCREMENT : Src1CtrlEnum_defaultEncoding_type := "10";
  constant Src1CtrlEnum_defaultEncoding_URS1 : Src1CtrlEnum_defaultEncoding_type := "11";

end pkg_enum;

package body pkg_enum is
  function pkg_mux (sel : std_logic;one : AluCtrlEnum;zero : AluCtrlEnum) return AluCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : AluBitwiseCtrlEnum;zero : AluBitwiseCtrlEnum) return AluBitwiseCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : ShiftCtrlEnum;zero : ShiftCtrlEnum) return ShiftCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : EnvCtrlEnum;zero : EnvCtrlEnum) return EnvCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : BranchCtrlEnum;zero : BranchCtrlEnum) return BranchCtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : Src2CtrlEnum;zero : Src2CtrlEnum) return Src2CtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : Src1CtrlEnum;zero : Src1CtrlEnum) return Src1CtrlEnum is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

end pkg_enum;


library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

package pkg_scala2hdl is
  function pkg_extract (that : std_logic_vector; bitId : integer) return std_logic;
  function pkg_extract (that : std_logic_vector; base : unsigned; size : integer) return std_logic_vector;
  function pkg_cat (a : std_logic_vector; b : std_logic_vector) return std_logic_vector;
  function pkg_not (value : std_logic_vector) return std_logic_vector;
  function pkg_extract (that : unsigned; bitId : integer) return std_logic;
  function pkg_extract (that : unsigned; base : unsigned; size : integer) return unsigned;
  function pkg_cat (a : unsigned; b : unsigned) return unsigned;
  function pkg_not (value : unsigned) return unsigned;
  function pkg_extract (that : signed; bitId : integer) return std_logic;
  function pkg_extract (that : signed; base : unsigned; size : integer) return signed;
  function pkg_cat (a : signed; b : signed) return signed;
  function pkg_not (value : signed) return signed;


  function pkg_mux (sel : std_logic;one : std_logic;zero : std_logic) return std_logic;
  function pkg_mux (sel : std_logic;one : std_logic_vector;zero : std_logic_vector) return std_logic_vector;
  function pkg_mux (sel : std_logic;one : unsigned;zero : unsigned) return unsigned;
  function pkg_mux (sel : std_logic;one : signed;zero : signed) return signed;


  function pkg_toStdLogic (value : boolean) return std_logic;
  function pkg_toStdLogicVector (value : std_logic) return std_logic_vector;
  function pkg_toUnsigned(value : std_logic) return unsigned;
  function pkg_toSigned (value : std_logic) return signed;
  function pkg_stdLogicVector (lit : std_logic_vector) return std_logic_vector;
  function pkg_unsigned (lit : unsigned) return unsigned;
  function pkg_signed (lit : signed) return signed;

  function pkg_resize (that : std_logic_vector; width : integer) return std_logic_vector;
  function pkg_resize (that : unsigned; width : integer) return unsigned;
  function pkg_resize (that : signed; width : integer) return signed;

  function pkg_extract (that : std_logic_vector; high : integer; low : integer) return std_logic_vector;
  function pkg_extract (that : unsigned; high : integer; low : integer) return unsigned;
  function pkg_extract (that : signed; high : integer; low : integer) return signed;

  function pkg_shiftRight (that : std_logic_vector; size : natural) return std_logic_vector;
  function pkg_shiftRight (that : std_logic_vector; size : unsigned) return std_logic_vector;
  function pkg_shiftLeft (that : std_logic_vector; size : natural) return std_logic_vector;
  function pkg_shiftLeft (that : std_logic_vector; size : unsigned) return std_logic_vector;

  function pkg_shiftRight (that : unsigned; size : natural) return unsigned;
  function pkg_shiftRight (that : unsigned; size : unsigned) return unsigned;
  function pkg_shiftLeft (that : unsigned; size : natural) return unsigned;
  function pkg_shiftLeft (that : unsigned; size : unsigned) return unsigned;

  function pkg_shiftRight (that : signed; size : natural) return signed;
  function pkg_shiftRight (that : signed; size : unsigned) return signed;
  function pkg_shiftLeft (that : signed; size : natural) return signed;
  function pkg_shiftLeft (that : signed; size : unsigned; w : integer) return signed;

  function pkg_rotateLeft (that : std_logic_vector; size : unsigned) return std_logic_vector;
end  pkg_scala2hdl;

package body pkg_scala2hdl is
  function pkg_extract (that : std_logic_vector; bitId : integer) return std_logic is
  begin
    return that(bitId);
  end pkg_extract;


  function pkg_extract (that : std_logic_vector; base : unsigned; size : integer) return std_logic_vector is
   constant elementCount : integer := (that'length-size)+1;
   type tableType is array (0 to elementCount-1) of std_logic_vector(size-1 downto 0);
   variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := that(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;


  function pkg_cat (a : std_logic_vector; b : std_logic_vector) return std_logic_vector is
    variable cat : std_logic_vector(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;


  function pkg_not (value : std_logic_vector) return std_logic_vector is
    variable ret : std_logic_vector(value'high downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;


  function pkg_extract (that : unsigned; bitId : integer) return std_logic is
  begin
    return that(bitId);
  end pkg_extract;


  function pkg_extract (that : unsigned; base : unsigned; size : integer) return unsigned is
   constant elementCount : integer := (that'length-size)+1;
   type tableType is array (0 to elementCount-1) of unsigned(size-1 downto 0);
   variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := that(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;


  function pkg_cat (a : unsigned; b : unsigned) return unsigned is
    variable cat : unsigned(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;


  function pkg_not (value : unsigned) return unsigned is
    variable ret : unsigned(value'high downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;


  function pkg_extract (that : signed; bitId : integer) return std_logic is
  begin
    return that(bitId);
  end pkg_extract;


  function pkg_extract (that : signed; base : unsigned; size : integer) return signed is
   constant elementCount : integer := (that'length-size)+1;
   type tableType is array (0 to elementCount-1) of signed(size-1 downto 0);
   variable table : tableType;
  begin
    for i in 0 to elementCount-1 loop
      table(i) := that(i + size - 1 downto i);
    end loop;
    return table(to_integer(base));
  end pkg_extract;


  function pkg_cat (a : signed; b : signed) return signed is
    variable cat : signed(a'length + b'length-1 downto 0);
  begin
    cat := a & b;
    return cat;
  end pkg_cat;


  function pkg_not (value : signed) return signed is
    variable ret : signed(value'high downto 0);
  begin
    ret := not value;
    return ret;
  end pkg_not;



  -- unsigned shifts
  function pkg_shiftRight (that : unsigned; size : natural) return unsigned is
  begin
    if size >= that'length then
      return "";
    else
      return shift_right(that,size)(that'high-size downto 0);
    end if;
  end pkg_shiftRight;

  function pkg_shiftRight (that : unsigned; size : unsigned) return unsigned is
  begin
    return shift_right(that,to_integer(size));
  end pkg_shiftRight;

  function pkg_shiftLeft (that : unsigned; size : natural) return unsigned is
  begin
    return shift_left(resize(that,that'length + size),size);
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : unsigned; size : unsigned) return unsigned is
  begin
    return shift_left(resize(that,that'length + 2**size'length - 1),to_integer(size));
  end pkg_shiftLeft;


  -- std_logic_vector shifts
  function pkg_shiftRight (that : std_logic_vector; size : natural) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftRight (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftLeft (that : std_logic_vector; size : natural) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  -- signed shifts
  function pkg_shiftRight (that : signed; size : natural) return signed is
  begin
    return signed(pkg_shiftRight(unsigned(that),size));
  end pkg_shiftRight;

  function pkg_shiftRight (that : signed; size : unsigned) return signed is
  begin
    return shift_right(that,to_integer(size));
  end pkg_shiftRight;

  function pkg_shiftLeft (that : signed; size : natural) return signed is
  begin
    return signed(pkg_shiftLeft(unsigned(that),size));
  end pkg_shiftLeft;

  function pkg_shiftLeft (that : signed; size : unsigned; w : integer) return signed is
  begin
    return shift_left(resize(that,w),to_integer(size));
  end pkg_shiftLeft;

  function pkg_rotateLeft (that : std_logic_vector; size : unsigned) return std_logic_vector is
  begin
    return std_logic_vector(rotate_left(unsigned(that),to_integer(size)));
  end pkg_rotateLeft;

  function pkg_extract (that : std_logic_vector; high : integer; low : integer) return std_logic_vector is
    variable temp : std_logic_vector(high-low downto 0);
  begin
    temp := that(high downto low);
    return temp;
  end pkg_extract;

  function pkg_extract (that : unsigned; high : integer; low : integer) return unsigned is
    variable temp : unsigned(high-low downto 0);
  begin
    temp := that(high downto low);
    return temp;
  end pkg_extract;

  function pkg_extract (that : signed; high : integer; low : integer) return signed is
    variable temp : signed(high-low downto 0);
  begin
    temp := that(high downto low);
    return temp;
  end pkg_extract;

  function pkg_mux (sel : std_logic;one : std_logic;zero : std_logic) return std_logic is
  begin
    if sel = '1' then
      return one;
    else
      return zero;
    end if;
  end pkg_mux;

  function pkg_mux (sel : std_logic;one : std_logic_vector;zero : std_logic_vector) return std_logic_vector is
    variable ret : std_logic_vector(zero'range);  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;  end pkg_mux;

  function pkg_mux (sel : std_logic;one : unsigned;zero : unsigned) return unsigned is
    variable ret : unsigned(zero'range);  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;  end pkg_mux;

  function pkg_mux (sel : std_logic;one : signed;zero : signed) return signed is
    variable ret : signed(zero'range);  begin
    if sel = '1' then
      ret := one;
    else
      ret := zero;
    end if;
    return ret;  end pkg_mux;

  function pkg_toStdLogic (value : boolean) return std_logic is
  begin
    if value = true then
      return '1';
    else
      return '0';
    end if;
  end pkg_toStdLogic;

  function pkg_toStdLogicVector (value : std_logic) return std_logic_vector is
    variable ret : std_logic_vector(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toStdLogicVector;

  function pkg_toUnsigned (value : std_logic) return unsigned is
    variable ret : unsigned(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toUnsigned;

  function pkg_toSigned (value : std_logic) return signed is
    variable ret : signed(0 downto 0);
  begin
    ret(0) := value;
    return ret;
  end pkg_toSigned;

  function pkg_stdLogicVector (lit : std_logic_vector) return std_logic_vector is
    variable ret : std_logic_vector(lit'length-1 downto 0);
  begin
    ret := lit;    return ret;
  end pkg_stdLogicVector;

  function pkg_unsigned (lit : unsigned) return unsigned is
    variable ret : unsigned(lit'length-1 downto 0);
  begin
    ret := lit;    return ret;
  end pkg_unsigned;

  function pkg_signed (lit : signed) return signed is
    variable ret : signed(lit'length-1 downto 0);
  begin
    ret := lit;    return ret;
  end pkg_signed;

  function pkg_resize (that : std_logic_vector; width : integer) return std_logic_vector is
  begin
    return std_logic_vector(resize(unsigned(that),width));
  end pkg_resize;


  function pkg_resize (that : unsigned; width : integer) return unsigned is
	  variable ret : unsigned(width-1 downto 0);
  begin
    if that'length = 0 then
       ret := (others => '0');
    else
       ret := resize(that,width);
    end if;
		return ret;
  end pkg_resize;
 
  function pkg_resize (that : signed; width : integer) return signed is
	  variable ret : signed(width-1 downto 0);
  begin
    if that'length = 0 then
       ret := (others => '0');
    elsif that'length >= width then
       ret := that(width-1 downto 0);
    else
       ret := resize(that,width);
    end if;
		return ret;
  end pkg_resize;
 end pkg_scala2hdl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.pkg_scala2hdl.all;
use work.all;
use work.pkg_enum.all;


entity StreamFifoLowLatency is
  port(
    io_push_valid : in std_logic;
    io_push_ready : out std_logic;
    io_push_payload_error : in std_logic;
    io_push_payload_inst : in std_logic_vector(31 downto 0);
    io_pop_valid : out std_logic;
    io_pop_ready : in std_logic;
    io_pop_payload_error : out std_logic;
    io_pop_payload_inst : out std_logic_vector(31 downto 0);
    io_flush : in std_logic;
    io_occupancy : out unsigned(0 downto 0);
    clk : in std_logic;
    reset : in std_logic
  );
end StreamFifoLowLatency;

architecture arch of StreamFifoLowLatency is
  signal zz_4 : std_logic;
  signal zz_5 : std_logic;
  signal zz_6 : std_logic;

  signal zz_1 : std_logic;
  signal pushPtr_willIncrement : std_logic;
  signal pushPtr_willClear : std_logic;
  signal pushPtr_willOverflowIfInc : std_logic;
  signal pushPtr_willOverflow : std_logic;
  signal popPtr_willIncrement : std_logic;
  signal popPtr_willClear : std_logic;
  signal popPtr_willOverflowIfInc : std_logic;
  signal popPtr_willOverflow : std_logic;
  signal ptrMatch : std_logic;
  signal risingOccupancy : std_logic;
  signal empty : std_logic;
  signal full : std_logic;
  signal pushing : std_logic;
  signal popping : std_logic;
  signal zz_2 : std_logic_vector(32 downto 0);
  signal zz_3 : std_logic_vector(32 downto 0);
begin
  io_push_ready <= zz_4;
  io_pop_valid <= zz_5;
  zz_6 <= (not empty);
  process(pushing)
  begin
    zz_1 <= pkg_toStdLogic(false);
    if pushing = '1' then
      zz_1 <= pkg_toStdLogic(true);
    end if;
  end process;

  process(pushing)
  begin
    pushPtr_willIncrement <= pkg_toStdLogic(false);
    if pushing = '1' then
      pushPtr_willIncrement <= pkg_toStdLogic(true);
    end if;
  end process;

  process(io_flush)
  begin
    pushPtr_willClear <= pkg_toStdLogic(false);
    if io_flush = '1' then
      pushPtr_willClear <= pkg_toStdLogic(true);
    end if;
  end process;

  pushPtr_willOverflowIfInc <= pkg_toStdLogic(true);
  pushPtr_willOverflow <= (pushPtr_willOverflowIfInc and pushPtr_willIncrement);
  process(popping)
  begin
    popPtr_willIncrement <= pkg_toStdLogic(false);
    if popping = '1' then
      popPtr_willIncrement <= pkg_toStdLogic(true);
    end if;
  end process;

  process(io_flush)
  begin
    popPtr_willClear <= pkg_toStdLogic(false);
    if io_flush = '1' then
      popPtr_willClear <= pkg_toStdLogic(true);
    end if;
  end process;

  popPtr_willOverflowIfInc <= pkg_toStdLogic(true);
  popPtr_willOverflow <= (popPtr_willOverflowIfInc and popPtr_willIncrement);
  ptrMatch <= pkg_toStdLogic(true);
  empty <= (ptrMatch and (not risingOccupancy));
  full <= (ptrMatch and risingOccupancy);
  pushing <= (io_push_valid and zz_4);
  popping <= (zz_5 and io_pop_ready);
  zz_4 <= (not full);
  process(zz_6,io_push_valid)
  begin
    if zz_6 = '1' then
      zz_5 <= pkg_toStdLogic(true);
    else
      zz_5 <= io_push_valid;
    end if;
  end process;

  zz_2 <= zz_3;
  process(zz_6,zz_2,io_push_payload_error)
  begin
    if zz_6 = '1' then
      io_pop_payload_error <= pkg_extract(pkg_extract(zz_2,0,0),0);
    else
      io_pop_payload_error <= io_push_payload_error;
    end if;
  end process;

  process(zz_6,zz_2,io_push_payload_inst)
  begin
    if zz_6 = '1' then
      io_pop_payload_inst <= pkg_extract(zz_2,32,1);
    else
      io_pop_payload_inst <= io_push_payload_inst;
    end if;
  end process;

  io_occupancy <= unsigned(pkg_toStdLogicVector((risingOccupancy and ptrMatch)));
  process(clk, reset)
  begin
    if reset = '1' then
      risingOccupancy <= pkg_toStdLogic(false);
    elsif rising_edge(clk) then
      if pkg_toStdLogic(pushing /= popping) = '1' then
        risingOccupancy <= pushing;
      end if;
      if io_flush = '1' then
        risingOccupancy <= pkg_toStdLogic(false);
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if zz_1 = '1' then
        zz_3 <= pkg_cat(io_push_payload_inst,pkg_toStdLogicVector(io_push_payload_error));
      end if;
    end if;
  end process;

end arch;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.pkg_scala2hdl.all;
use work.all;
use work.pkg_enum.all;


entity VexRiscv is
  port(
    iBus_cmd_valid : out std_logic;
    iBus_cmd_ready : in std_logic;
    iBus_cmd_payload_pc : out unsigned(31 downto 0);
    iBus_rsp_valid : in std_logic;
    iBus_rsp_payload_error : in std_logic;
    iBus_rsp_payload_inst : in std_logic_vector(31 downto 0);
    timerInterrupt : in std_logic;
    externalInterrupt : in std_logic;
    softwareInterrupt : in std_logic;
    dBus_cmd_valid : out std_logic;
    dBus_cmd_ready : in std_logic;
    dBus_cmd_payload_wr : out std_logic;
    dBus_cmd_payload_address : out unsigned(31 downto 0);
    dBus_cmd_payload_data : out std_logic_vector(31 downto 0);
    dBus_cmd_payload_size : out unsigned(1 downto 0);
    dBus_rsp_ready : in std_logic;
    dBus_rsp_error : in std_logic;
    dBus_rsp_data : in std_logic_vector(31 downto 0);
    clk : in std_logic;
    reset : in std_logic
  );
end VexRiscv;

architecture arch of VexRiscv is
  signal zz_103 : std_logic_vector(31 downto 0);
  signal zz_104 : std_logic_vector(31 downto 0);
  signal zz_105 : unsigned(31 downto 0);
  signal zz_106 : unsigned(1 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy : unsigned(0 downto 0);
  signal zz_107 : std_logic;
  signal zz_108 : std_logic;
  signal zz_109 : std_logic;
  signal zz_110 : std_logic;
  signal zz_111 : std_logic;
  signal zz_112 : std_logic_vector(1 downto 0);
  signal zz_113 : std_logic;
  signal zz_114 : std_logic;
  signal zz_115 : std_logic;
  signal zz_116 : std_logic;
  signal zz_117 : std_logic;
  signal zz_118 : std_logic;
  signal zz_119 : std_logic;
  signal zz_120 : std_logic;
  signal zz_121 : std_logic;
  signal zz_122 : std_logic;
  signal zz_123 : std_logic_vector(1 downto 0);
  signal zz_124 : std_logic;
  signal zz_125 : std_logic;
  signal zz_126 : std_logic;
  signal zz_127 : std_logic_vector(31 downto 0);
  signal zz_128 : std_logic_vector(31 downto 0);
  signal zz_129 : std_logic_vector(31 downto 0);
  signal zz_130 : std_logic_vector(31 downto 0);
  signal zz_131 : std_logic;
  signal zz_132 : std_logic_vector(0 downto 0);
  signal zz_133 : std_logic_vector(0 downto 0);
  signal zz_134 : std_logic;
  signal zz_135 : std_logic_vector(0 downto 0);
  signal zz_136 : std_logic_vector(18 downto 0);
  signal zz_137 : std_logic_vector(31 downto 0);
  signal zz_138 : std_logic;
  signal zz_139 : std_logic;
  signal zz_140 : std_logic_vector(0 downto 0);
  signal zz_141 : std_logic_vector(0 downto 0);
  signal zz_142 : std_logic_vector(1 downto 0);
  signal zz_143 : std_logic_vector(1 downto 0);
  signal zz_144 : std_logic;
  signal zz_145 : std_logic_vector(0 downto 0);
  signal zz_146 : std_logic_vector(15 downto 0);
  signal zz_147 : std_logic_vector(31 downto 0);
  signal zz_148 : std_logic_vector(31 downto 0);
  signal zz_149 : std_logic_vector(31 downto 0);
  signal zz_150 : std_logic;
  signal zz_151 : std_logic;
  signal zz_152 : std_logic_vector(0 downto 0);
  signal zz_153 : std_logic_vector(1 downto 0);
  signal zz_154 : std_logic_vector(1 downto 0);
  signal zz_155 : std_logic_vector(1 downto 0);
  signal zz_156 : std_logic;
  signal zz_157 : std_logic_vector(0 downto 0);
  signal zz_158 : std_logic_vector(12 downto 0);
  signal zz_159 : std_logic_vector(31 downto 0);
  signal zz_160 : std_logic_vector(31 downto 0);
  signal zz_161 : std_logic_vector(31 downto 0);
  signal zz_162 : std_logic_vector(31 downto 0);
  signal zz_163 : std_logic_vector(31 downto 0);
  signal zz_164 : std_logic_vector(31 downto 0);
  signal zz_165 : std_logic_vector(31 downto 0);
  signal zz_166 : std_logic_vector(31 downto 0);
  signal zz_167 : std_logic_vector(31 downto 0);
  signal zz_168 : std_logic;
  signal zz_169 : std_logic;
  signal zz_170 : std_logic_vector(0 downto 0);
  signal zz_171 : std_logic_vector(0 downto 0);
  signal zz_172 : std_logic_vector(0 downto 0);
  signal zz_173 : std_logic_vector(0 downto 0);
  signal zz_174 : std_logic;
  signal zz_175 : std_logic_vector(0 downto 0);
  signal zz_176 : std_logic_vector(9 downto 0);
  signal zz_177 : std_logic_vector(31 downto 0);
  signal zz_178 : std_logic_vector(31 downto 0);
  signal zz_179 : std_logic_vector(31 downto 0);
  signal zz_180 : std_logic;
  signal zz_181 : std_logic;
  signal zz_182 : std_logic_vector(0 downto 0);
  signal zz_183 : std_logic_vector(0 downto 0);
  signal zz_184 : std_logic;
  signal zz_185 : std_logic_vector(0 downto 0);
  signal zz_186 : std_logic_vector(6 downto 0);
  signal zz_187 : std_logic_vector(31 downto 0);
  signal zz_188 : std_logic_vector(31 downto 0);
  signal zz_189 : std_logic;
  signal zz_190 : std_logic_vector(0 downto 0);
  signal zz_191 : std_logic_vector(0 downto 0);
  signal zz_192 : std_logic_vector(31 downto 0);
  signal zz_193 : std_logic_vector(31 downto 0);
  signal zz_194 : std_logic_vector(0 downto 0);
  signal zz_195 : std_logic_vector(4 downto 0);
  signal zz_196 : std_logic_vector(1 downto 0);
  signal zz_197 : std_logic_vector(1 downto 0);
  signal zz_198 : std_logic;
  signal zz_199 : std_logic_vector(0 downto 0);
  signal zz_200 : std_logic_vector(2 downto 0);
  signal zz_201 : std_logic_vector(31 downto 0);
  signal zz_202 : std_logic_vector(31 downto 0);
  signal zz_203 : std_logic_vector(31 downto 0);
  signal zz_204 : std_logic;
  signal zz_205 : std_logic_vector(0 downto 0);
  signal zz_206 : std_logic_vector(1 downto 0);
  signal zz_207 : std_logic_vector(31 downto 0);
  signal zz_208 : std_logic_vector(31 downto 0);
  signal zz_209 : std_logic_vector(31 downto 0);
  signal zz_210 : std_logic_vector(31 downto 0);
  signal zz_211 : std_logic;
  signal zz_212 : std_logic_vector(0 downto 0);
  signal zz_213 : std_logic_vector(0 downto 0);
  signal zz_214 : std_logic;
  signal zz_215 : std_logic;
  signal zz_216 : std_logic_vector(31 downto 0);
  signal zz_217 : std_logic_vector(31 downto 0);
  signal zz_218 : std_logic_vector(31 downto 0);
  signal zz_219 : std_logic_vector(31 downto 0);
  signal zz_220 : std_logic_vector(31 downto 0);
  signal zz_221 : std_logic;
  signal zz_222 : std_logic;

  signal decode_SRC2 : std_logic_vector(31 downto 0);
  signal memory_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal decode_CSR_WRITE_OPCODE : std_logic;
  signal writeBack_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal memory_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal execute_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal decode_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal execute_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal decode_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal decode_CSR_READ_OPCODE : std_logic;
  signal decode_ALU_CTRL : AluCtrlEnum_defaultEncoding_type;
  signal zz_1 : AluCtrlEnum_defaultEncoding_type;
  signal zz_2 : AluCtrlEnum_defaultEncoding_type;
  signal zz_3 : AluCtrlEnum_defaultEncoding_type;
  signal decode_RS1 : std_logic_vector(31 downto 0);
  signal decode_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal decode_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_4 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_5 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_6 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal decode_SHIFT_CTRL : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_7 : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_8 : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_9 : ShiftCtrlEnum_defaultEncoding_type;
  signal decode_MEMORY_ENABLE : std_logic;
  signal decode_SRC1 : std_logic_vector(31 downto 0);
  signal writeBack_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal execute_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal decode_MEMORY_STORE : std_logic;
  signal zz_10 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_11 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_12 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_13 : EnvCtrlEnum_defaultEncoding_type;
  signal decode_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal zz_14 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_15 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_16 : EnvCtrlEnum_defaultEncoding_type;
  signal decode_SRC_LESS_UNSIGNED : std_logic;
  signal decode_BRANCH_CTRL : BranchCtrlEnum_defaultEncoding_type;
  signal zz_17 : BranchCtrlEnum_defaultEncoding_type;
  signal zz_18 : BranchCtrlEnum_defaultEncoding_type;
  signal zz_19 : BranchCtrlEnum_defaultEncoding_type;
  signal decode_IS_CSR : std_logic;
  signal execute_BRANCH_DO : std_logic;
  signal decode_SRC2_FORCE_ZERO : std_logic;
  signal decode_RS2 : std_logic_vector(31 downto 0);
  signal memory_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal execute_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal execute_BRANCH_CALC : unsigned(31 downto 0);
  signal memory_PC : unsigned(31 downto 0);
  signal memory_BRANCH_CALC : unsigned(31 downto 0);
  signal memory_BRANCH_DO : std_logic;
  signal execute_PC : unsigned(31 downto 0);
  signal execute_RS1 : std_logic_vector(31 downto 0);
  signal execute_BRANCH_CTRL : BranchCtrlEnum_defaultEncoding_type;
  signal zz_20 : BranchCtrlEnum_defaultEncoding_type;
  signal decode_RS2_USE : std_logic;
  signal decode_RS1_USE : std_logic;
  signal execute_REGFILE_WRITE_VALID : std_logic;
  signal execute_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal memory_REGFILE_WRITE_VALID : std_logic;
  signal memory_INSTRUCTION : std_logic_vector(31 downto 0);
  signal memory_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal writeBack_REGFILE_WRITE_VALID : std_logic;
  signal memory_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal execute_SHIFT_CTRL : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_21 : ShiftCtrlEnum_defaultEncoding_type;
  signal execute_SRC_LESS_UNSIGNED : std_logic;
  signal execute_SRC2_FORCE_ZERO : std_logic;
  signal execute_SRC_USE_SUB_LESS : std_logic;
  signal zz_22 : unsigned(31 downto 0);
  signal zz_23 : std_logic_vector(31 downto 0);
  signal decode_SRC2_CTRL : Src2CtrlEnum_defaultEncoding_type;
  signal zz_24 : Src2CtrlEnum_defaultEncoding_type;
  signal zz_25 : std_logic_vector(31 downto 0);
  signal decode_SRC1_CTRL : Src1CtrlEnum_defaultEncoding_type;
  signal zz_26 : Src1CtrlEnum_defaultEncoding_type;
  signal decode_SRC_USE_SUB_LESS : std_logic;
  signal decode_SRC_ADD_ZERO : std_logic;
  signal execute_SRC_ADD_SUB : std_logic_vector(31 downto 0);
  signal execute_SRC_LESS : std_logic;
  signal execute_ALU_CTRL : AluCtrlEnum_defaultEncoding_type;
  signal zz_27 : AluCtrlEnum_defaultEncoding_type;
  signal execute_SRC2 : std_logic_vector(31 downto 0);
  signal execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_28 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_29 : std_logic_vector(31 downto 0);
  signal zz_30 : std_logic;
  signal zz_31 : std_logic;
  signal decode_INSTRUCTION_ANTICIPATED : std_logic_vector(31 downto 0);
  signal decode_REGFILE_WRITE_VALID : std_logic;
  signal zz_32 : BranchCtrlEnum_defaultEncoding_type;
  signal zz_33 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_34 : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_35 : Src2CtrlEnum_defaultEncoding_type;
  signal zz_36 : AluCtrlEnum_defaultEncoding_type;
  signal zz_37 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_38 : Src1CtrlEnum_defaultEncoding_type;
  signal zz_39 : std_logic_vector(31 downto 0);
  signal execute_SRC1 : std_logic_vector(31 downto 0);
  signal execute_CSR_READ_OPCODE : std_logic;
  signal execute_CSR_WRITE_OPCODE : std_logic;
  signal execute_IS_CSR : std_logic;
  signal memory_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal zz_40 : EnvCtrlEnum_defaultEncoding_type;
  signal execute_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal zz_41 : EnvCtrlEnum_defaultEncoding_type;
  signal writeBack_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal zz_42 : EnvCtrlEnum_defaultEncoding_type;
  signal writeBack_MEMORY_STORE : std_logic;
  signal zz_43 : std_logic_vector(31 downto 0);
  signal writeBack_MEMORY_ENABLE : std_logic;
  signal writeBack_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal writeBack_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal memory_MEMORY_STORE : std_logic;
  signal memory_MEMORY_ENABLE : std_logic;
  signal execute_SRC_ADD : std_logic_vector(31 downto 0);
  signal execute_RS2 : std_logic_vector(31 downto 0);
  signal execute_INSTRUCTION : std_logic_vector(31 downto 0);
  signal execute_MEMORY_STORE : std_logic;
  signal execute_MEMORY_ENABLE : std_logic;
  signal execute_ALIGNEMENT_FAULT : std_logic;
  signal zz_44 : unsigned(31 downto 0);
  signal decode_PC : unsigned(31 downto 0);
  signal decode_INSTRUCTION : std_logic_vector(31 downto 0);
  signal writeBack_PC : unsigned(31 downto 0);
  signal writeBack_INSTRUCTION : std_logic_vector(31 downto 0);
  signal decode_arbitration_haltItself : std_logic;
  signal decode_arbitration_haltByOther : std_logic;
  signal decode_arbitration_removeIt : std_logic;
  signal decode_arbitration_flushIt : std_logic;
  signal decode_arbitration_flushNext : std_logic;
  signal decode_arbitration_isValid : std_logic;
  signal decode_arbitration_isStuck : std_logic;
  signal decode_arbitration_isStuckByOthers : std_logic;
  signal decode_arbitration_isFlushed : std_logic;
  signal decode_arbitration_isMoving : std_logic;
  signal decode_arbitration_isFiring : std_logic;
  signal execute_arbitration_haltItself : std_logic;
  signal execute_arbitration_haltByOther : std_logic;
  signal execute_arbitration_removeIt : std_logic;
  signal execute_arbitration_flushIt : std_logic;
  signal execute_arbitration_flushNext : std_logic;
  signal execute_arbitration_isValid : std_logic;
  signal execute_arbitration_isStuck : std_logic;
  signal execute_arbitration_isStuckByOthers : std_logic;
  signal execute_arbitration_isFlushed : std_logic;
  signal execute_arbitration_isMoving : std_logic;
  signal execute_arbitration_isFiring : std_logic;
  signal memory_arbitration_haltItself : std_logic;
  signal memory_arbitration_haltByOther : std_logic;
  signal memory_arbitration_removeIt : std_logic;
  signal memory_arbitration_flushIt : std_logic;
  signal memory_arbitration_flushNext : std_logic;
  signal memory_arbitration_isValid : std_logic;
  signal memory_arbitration_isStuck : std_logic;
  signal memory_arbitration_isStuckByOthers : std_logic;
  signal memory_arbitration_isFlushed : std_logic;
  signal memory_arbitration_isMoving : std_logic;
  signal memory_arbitration_isFiring : std_logic;
  signal writeBack_arbitration_haltItself : std_logic;
  signal writeBack_arbitration_haltByOther : std_logic;
  signal writeBack_arbitration_removeIt : std_logic;
  signal writeBack_arbitration_flushIt : std_logic;
  signal writeBack_arbitration_flushNext : std_logic;
  signal writeBack_arbitration_isValid : std_logic;
  signal writeBack_arbitration_isStuck : std_logic;
  signal writeBack_arbitration_isStuckByOthers : std_logic;
  signal writeBack_arbitration_isFlushed : std_logic;
  signal writeBack_arbitration_isMoving : std_logic;
  signal writeBack_arbitration_isFiring : std_logic;
  signal lastStageInstruction : std_logic_vector(31 downto 0);
  signal lastStagePc : unsigned(31 downto 0);
  signal lastStageIsValid : std_logic;
  signal lastStageIsFiring : std_logic;
  signal IBusSimplePlugin_fetcherHalt : std_logic;
  signal IBusSimplePlugin_fetcherflushIt : std_logic;
  signal IBusSimplePlugin_incomingInstruction : std_logic;
  signal IBusSimplePlugin_pcValids_0 : std_logic;
  signal IBusSimplePlugin_pcValids_1 : std_logic;
  signal IBusSimplePlugin_pcValids_2 : std_logic;
  signal IBusSimplePlugin_pcValids_3 : std_logic;
  signal CsrPlugin_inWfi : std_logic;
  signal CsrPlugin_thirdPartyWake : std_logic;
  signal CsrPlugin_jumpInterface_valid : std_logic;
  signal CsrPlugin_jumpInterface_payload : unsigned(31 downto 0);
  signal CsrPlugin_exceptionPendings_0 : std_logic;
  signal CsrPlugin_exceptionPendings_1 : std_logic;
  signal CsrPlugin_exceptionPendings_2 : std_logic;
  signal CsrPlugin_exceptionPendings_3 : std_logic;
  signal contextSwitching : std_logic;
  signal CsrPlugin_privilege : unsigned(1 downto 0);
  signal CsrPlugin_forceMachineWire : std_logic;
  signal CsrPlugin_allowInterrupts : std_logic;
  signal CsrPlugin_allowException : std_logic;
  signal BranchPlugin_jumpInterface_valid : std_logic;
  signal BranchPlugin_jumpInterface_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_jump_pcLoad_valid : std_logic;
  signal IBusSimplePlugin_jump_pcLoad_payload : unsigned(31 downto 0);
  signal zz_45 : unsigned(1 downto 0);
  signal IBusSimplePlugin_fetchPc_output_valid : std_logic;
  signal IBusSimplePlugin_fetchPc_output_ready : std_logic;
  signal IBusSimplePlugin_fetchPc_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_fetchPc_pcReg : unsigned(31 downto 0);
  signal IBusSimplePlugin_fetchPc_corrected : std_logic;
  signal IBusSimplePlugin_fetchPc_pcRegPropagate : std_logic;
  signal IBusSimplePlugin_fetchPc_booted : std_logic;
  signal IBusSimplePlugin_fetchPc_inc : std_logic;
  signal IBusSimplePlugin_fetchPc_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_0_input_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_input_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_input_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_0_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_0_halt : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_0_inputSample : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_input_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_1_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_output_payload : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_stages_1_halt : std_logic;
  signal IBusSimplePlugin_iBusRsp_stages_1_inputSample : std_logic;
  signal zz_46 : std_logic;
  signal zz_47 : std_logic;
  signal zz_48 : std_logic;
  signal zz_49 : std_logic;
  signal zz_50 : std_logic;
  signal IBusSimplePlugin_iBusRsp_readyForError : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_valid : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_ready : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_output_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_iBusRsp_output_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_iBusRsp_output_payload_isRvc : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_valid : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_ready : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_injector_decodeInput_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_injector_decodeInput_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_injector_decodeInput_payload_isRvc : std_logic;
  signal zz_51 : std_logic;
  signal zz_52 : unsigned(31 downto 0);
  signal zz_53 : std_logic;
  signal zz_54 : std_logic_vector(31 downto 0);
  signal zz_55 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_0 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_1 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_2 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_3 : std_logic;
  signal IBusSimplePlugin_injector_nextPcCalc_valids_4 : std_logic;
  signal IBusSimplePlugin_injector_decodeRemoved : std_logic;
  signal IBusSimplePlugin_injector_formal_rawInDecode : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_cmd_valid : std_logic;
  signal IBusSimplePlugin_cmd_ready : std_logic;
  signal IBusSimplePlugin_cmd_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_pendingCmd : unsigned(2 downto 0);
  signal IBusSimplePlugin_pendingCmdNext : unsigned(2 downto 0);
  signal IBusSimplePlugin_rspJoin_discardCounter : unsigned(2 downto 0);
  signal IBusSimplePlugin_rspJoin_rspBufferOutput_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBufferOutput_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error : std_logic;
  signal IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst : std_logic_vector(31 downto 0);
  signal iBus_rsp_takeWhen_valid : std_logic;
  signal iBus_rsp_takeWhen_payload_error : std_logic;
  signal iBus_rsp_takeWhen_payload_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_fetchRsp_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_rspJoin_fetchRsp_rsp_error : std_logic;
  signal IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_fetchRsp_isRvc : std_logic;
  signal IBusSimplePlugin_rspJoin_join_valid : std_logic;
  signal IBusSimplePlugin_rspJoin_join_ready : std_logic;
  signal IBusSimplePlugin_rspJoin_join_payload_pc : unsigned(31 downto 0);
  signal IBusSimplePlugin_rspJoin_join_payload_rsp_error : std_logic;
  signal IBusSimplePlugin_rspJoin_join_payload_rsp_inst : std_logic_vector(31 downto 0);
  signal IBusSimplePlugin_rspJoin_join_payload_isRvc : std_logic;
  signal IBusSimplePlugin_rspJoin_exceptionDetected : std_logic;
  signal IBusSimplePlugin_rspJoin_redoRequired : std_logic;
  signal zz_56 : std_logic;
  signal zz_57 : std_logic;
  signal execute_DBusSimplePlugin_skipCmd : std_logic;
  signal zz_58 : std_logic_vector(31 downto 0);
  signal zz_59 : std_logic_vector(3 downto 0);
  signal execute_DBusSimplePlugin_formalMask : std_logic_vector(3 downto 0);
  signal writeBack_DBusSimplePlugin_rspShifted : std_logic_vector(31 downto 0);
  signal zz_60 : std_logic;
  signal zz_61 : std_logic_vector(31 downto 0);
  signal zz_62 : std_logic;
  signal zz_63 : std_logic_vector(31 downto 0);
  signal writeBack_DBusSimplePlugin_rspFormated : std_logic_vector(31 downto 0);
  signal CsrPlugin_misa_base : unsigned(1 downto 0);
  signal CsrPlugin_misa_extensions : std_logic_vector(25 downto 0);
  signal CsrPlugin_mtvec_mode : std_logic_vector(1 downto 0);
  signal CsrPlugin_mtvec_base : unsigned(29 downto 0);
  signal CsrPlugin_mepc : unsigned(31 downto 0);
  signal CsrPlugin_mstatus_MIE : std_logic;
  signal CsrPlugin_mstatus_MPIE : std_logic;
  signal CsrPlugin_mstatus_MPP : unsigned(1 downto 0);
  signal CsrPlugin_mip_MEIP : std_logic;
  signal CsrPlugin_mip_MTIP : std_logic;
  signal CsrPlugin_mip_MSIP : std_logic;
  signal CsrPlugin_mie_MEIE : std_logic;
  signal CsrPlugin_mie_MTIE : std_logic;
  signal CsrPlugin_mie_MSIE : std_logic;
  signal CsrPlugin_mcause_interrupt : std_logic;
  signal CsrPlugin_mcause_exceptionCode : unsigned(3 downto 0);
  signal CsrPlugin_mtval : unsigned(31 downto 0);
  signal CsrPlugin_mcycle : unsigned(63 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000";
  signal CsrPlugin_minstret : unsigned(63 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000";
  signal zz_64 : std_logic;
  signal zz_65 : std_logic;
  signal zz_66 : std_logic;
  signal CsrPlugin_interrupt_valid : std_logic;
  signal CsrPlugin_interrupt_code : unsigned(3 downto 0);
  signal CsrPlugin_interrupt_targetPrivilege : unsigned(1 downto 0);
  signal CsrPlugin_exception : std_logic;
  signal CsrPlugin_lastStageWasWfi : std_logic;
  signal CsrPlugin_pipelineLiberator_done : std_logic;
  signal CsrPlugin_interruptJump : std_logic;
  signal CsrPlugin_hadException : std_logic;
  signal CsrPlugin_targetPrivilege : unsigned(1 downto 0);
  signal CsrPlugin_trapCause : unsigned(3 downto 0);
  signal CsrPlugin_xtvec_mode : std_logic_vector(1 downto 0);
  signal CsrPlugin_xtvec_base : unsigned(29 downto 0);
  signal execute_CsrPlugin_wfiWake : std_logic;
  signal execute_CsrPlugin_blockedBySideEffects : std_logic;
  signal execute_CsrPlugin_illegalAccess : std_logic;
  signal execute_CsrPlugin_illegalInstruction : std_logic;
  signal execute_CsrPlugin_readData : std_logic_vector(31 downto 0);
  signal execute_CsrPlugin_writeInstruction : std_logic;
  signal execute_CsrPlugin_readInstruction : std_logic;
  signal execute_CsrPlugin_writeEnable : std_logic;
  signal execute_CsrPlugin_readEnable : std_logic;
  signal execute_CsrPlugin_readToWriteData : std_logic_vector(31 downto 0);
  signal execute_CsrPlugin_writeData : std_logic_vector(31 downto 0);
  signal execute_CsrPlugin_csrAddress : std_logic_vector(11 downto 0);
  signal zz_67 : std_logic_vector(24 downto 0);
  signal zz_68 : std_logic;
  signal zz_69 : std_logic;
  signal zz_70 : std_logic;
  signal zz_71 : std_logic;
  signal zz_72 : std_logic;
  signal zz_73 : Src1CtrlEnum_defaultEncoding_type;
  signal zz_74 : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal zz_75 : AluCtrlEnum_defaultEncoding_type;
  signal zz_76 : Src2CtrlEnum_defaultEncoding_type;
  signal zz_77 : ShiftCtrlEnum_defaultEncoding_type;
  signal zz_78 : EnvCtrlEnum_defaultEncoding_type;
  signal zz_79 : BranchCtrlEnum_defaultEncoding_type;
  signal decode_RegFilePlugin_regFileReadAddress1 : unsigned(4 downto 0);
  signal decode_RegFilePlugin_regFileReadAddress2 : unsigned(4 downto 0);
  signal decode_RegFilePlugin_rs1Data : std_logic_vector(31 downto 0);
  signal decode_RegFilePlugin_rs2Data : std_logic_vector(31 downto 0);
  signal lastStageRegFileWrite_valid : std_logic;
  signal lastStageRegFileWrite_payload_address : unsigned(4 downto 0);
  signal lastStageRegFileWrite_payload_data : std_logic_vector(31 downto 0);
  signal zz_80 : std_logic;
  signal execute_IntAluPlugin_bitwise : std_logic_vector(31 downto 0);
  signal zz_81 : std_logic_vector(31 downto 0);
  signal zz_82 : std_logic_vector(31 downto 0);
  signal zz_83 : std_logic;
  signal zz_84 : std_logic_vector(19 downto 0);
  signal zz_85 : std_logic;
  signal zz_86 : std_logic_vector(19 downto 0);
  signal zz_87 : std_logic_vector(31 downto 0);
  signal execute_SrcPlugin_addSub : std_logic_vector(31 downto 0);
  signal execute_SrcPlugin_less : std_logic;
  signal execute_LightShifterPlugin_isActive : std_logic;
  signal execute_LightShifterPlugin_isShift : std_logic;
  signal execute_LightShifterPlugin_amplitudeReg : unsigned(4 downto 0);
  signal execute_LightShifterPlugin_amplitude : unsigned(4 downto 0);
  signal execute_LightShifterPlugin_shiftInput : std_logic_vector(31 downto 0);
  signal execute_LightShifterPlugin_done : std_logic;
  signal zz_88 : std_logic_vector(31 downto 0);
  signal zz_89 : std_logic;
  signal zz_90 : std_logic;
  signal zz_91 : std_logic;
  signal zz_92 : std_logic_vector(4 downto 0);
  signal execute_BranchPlugin_eq : std_logic;
  signal zz_93 : std_logic_vector(2 downto 0);
  signal zz_94 : std_logic;
  signal zz_95 : std_logic;
  signal execute_BranchPlugin_branch_src1 : unsigned(31 downto 0);
  signal zz_96 : std_logic;
  signal zz_97 : std_logic_vector(10 downto 0);
  signal zz_98 : std_logic;
  signal zz_99 : std_logic_vector(19 downto 0);
  signal zz_100 : std_logic;
  signal zz_101 : std_logic_vector(18 downto 0);
  signal zz_102 : std_logic_vector(31 downto 0);
  signal execute_BranchPlugin_branch_src2 : unsigned(31 downto 0);
  signal execute_BranchPlugin_branchAdder : unsigned(31 downto 0);
  signal decode_to_execute_SRC_USE_SUB_LESS : std_logic;
  signal decode_to_execute_PC : unsigned(31 downto 0);
  signal execute_to_memory_PC : unsigned(31 downto 0);
  signal memory_to_writeBack_PC : unsigned(31 downto 0);
  signal execute_to_memory_BRANCH_CALC : unsigned(31 downto 0);
  signal execute_to_memory_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal memory_to_writeBack_MEMORY_ADDRESS_LOW : unsigned(1 downto 0);
  signal decode_to_execute_RS2 : std_logic_vector(31 downto 0);
  signal decode_to_execute_SRC2_FORCE_ZERO : std_logic;
  signal execute_to_memory_BRANCH_DO : std_logic;
  signal decode_to_execute_REGFILE_WRITE_VALID : std_logic;
  signal execute_to_memory_REGFILE_WRITE_VALID : std_logic;
  signal memory_to_writeBack_REGFILE_WRITE_VALID : std_logic;
  signal decode_to_execute_IS_CSR : std_logic;
  signal decode_to_execute_BRANCH_CTRL : BranchCtrlEnum_defaultEncoding_type;
  signal decode_to_execute_SRC_LESS_UNSIGNED : std_logic;
  signal decode_to_execute_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal execute_to_memory_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal memory_to_writeBack_ENV_CTRL : EnvCtrlEnum_defaultEncoding_type;
  signal decode_to_execute_MEMORY_STORE : std_logic;
  signal execute_to_memory_MEMORY_STORE : std_logic;
  signal memory_to_writeBack_MEMORY_STORE : std_logic;
  signal execute_to_memory_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal memory_to_writeBack_REGFILE_WRITE_DATA : std_logic_vector(31 downto 0);
  signal decode_to_execute_SRC1 : std_logic_vector(31 downto 0);
  signal decode_to_execute_MEMORY_ENABLE : std_logic;
  signal execute_to_memory_MEMORY_ENABLE : std_logic;
  signal memory_to_writeBack_MEMORY_ENABLE : std_logic;
  signal decode_to_execute_SHIFT_CTRL : ShiftCtrlEnum_defaultEncoding_type;
  signal decode_to_execute_ALU_BITWISE_CTRL : AluBitwiseCtrlEnum_defaultEncoding_type;
  signal decode_to_execute_BYPASSABLE_EXECUTE_STAGE : std_logic;
  signal decode_to_execute_RS1 : std_logic_vector(31 downto 0);
  signal decode_to_execute_ALU_CTRL : AluCtrlEnum_defaultEncoding_type;
  signal decode_to_execute_INSTRUCTION : std_logic_vector(31 downto 0);
  signal execute_to_memory_INSTRUCTION : std_logic_vector(31 downto 0);
  signal memory_to_writeBack_INSTRUCTION : std_logic_vector(31 downto 0);
  signal decode_to_execute_CSR_READ_OPCODE : std_logic;
  signal decode_to_execute_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal execute_to_memory_BYPASSABLE_MEMORY_STAGE : std_logic;
  signal decode_to_execute_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal execute_to_memory_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal memory_to_writeBack_FORMAL_PC_NEXT : unsigned(31 downto 0);
  signal decode_to_execute_CSR_WRITE_OPCODE : std_logic;
  signal memory_to_writeBack_MEMORY_READ_DATA : std_logic_vector(31 downto 0);
  signal decode_to_execute_SRC2 : std_logic_vector(31 downto 0);
  type RegFilePlugin_regFile_type is array (0 to 31) of std_logic_vector(31 downto 0);
  signal RegFilePlugin_regFile : RegFilePlugin_regFile_type;
begin
  dBus_cmd_payload_address <= zz_105;
  dBus_cmd_payload_size <= zz_106;
  zz_107 <= (execute_arbitration_isValid and execute_IS_CSR);
  zz_108 <= ((execute_arbitration_isValid and execute_LightShifterPlugin_isShift) and pkg_toStdLogic(pkg_extract(execute_SRC2,4,0) /= pkg_stdLogicVector("00000")));
  zz_109 <= (not execute_arbitration_isStuckByOthers);
  zz_110 <= (CsrPlugin_hadException or CsrPlugin_interruptJump);
  zz_111 <= (writeBack_arbitration_isValid and pkg_toStdLogic(writeBack_ENV_CTRL = EnvCtrlEnum_defaultEncoding_XRET));
  zz_112 <= pkg_extract(writeBack_INSTRUCTION,29,28);
  zz_113 <= (writeBack_arbitration_isValid and writeBack_REGFILE_WRITE_VALID);
  zz_114 <= (pkg_toStdLogic(true) or (not pkg_toStdLogic(true)));
  zz_115 <= (memory_arbitration_isValid and memory_REGFILE_WRITE_VALID);
  zz_116 <= (pkg_toStdLogic(true) or (not memory_BYPASSABLE_MEMORY_STAGE));
  zz_117 <= (execute_arbitration_isValid and execute_REGFILE_WRITE_VALID);
  zz_118 <= (pkg_toStdLogic(true) or (not execute_BYPASSABLE_EXECUTE_STAGE));
  zz_119 <= (CsrPlugin_mstatus_MIE or pkg_toStdLogic(CsrPlugin_privilege < pkg_unsigned("11")));
  zz_120 <= ((zz_64 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  zz_121 <= ((zz_65 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  zz_122 <= ((zz_66 and pkg_toStdLogic(true)) and (not pkg_toStdLogic(false)));
  zz_123 <= pkg_extract(writeBack_INSTRUCTION,13,12);
  zz_124 <= pkg_extract(execute_INSTRUCTION,13);
  zz_125 <= pkg_toStdLogic(true);
  zz_126 <= pkg_toStdLogic(true);
  zz_127 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000011100"));
  zz_128 <= pkg_stdLogicVector("00000000000000000000000000000100");
  zz_129 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001011000"));
  zz_130 <= pkg_stdLogicVector("00000000000000000000000001000000");
  zz_131 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000011000001010000")) = pkg_stdLogicVector("00000000000000000000000001010000"));
  zz_132 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_137) = pkg_stdLogicVector("00000000000000000101000000010000")));
  zz_133 <= pkg_stdLogicVector("0");
  zz_134 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_138),pkg_toStdLogicVector(zz_139)) /= pkg_stdLogicVector("00"));
  zz_135 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_140,zz_141) /= pkg_stdLogicVector("00")));
  zz_136 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_142 /= zz_143)),pkg_cat(pkg_toStdLogicVector(zz_144),pkg_cat(zz_145,zz_146)));
  zz_137 <= pkg_stdLogicVector("00000000000000000111000001010100");
  zz_138 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("01000000000000000011000001010100")) = pkg_stdLogicVector("01000000000000000001000000010000"));
  zz_139 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000111000001010100")) = pkg_stdLogicVector("00000000000000000001000000010000"));
  zz_140 <= pkg_toStdLogicVector(zz_72);
  zz_141 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_147) = pkg_stdLogicVector("00000000000000000000000000100000")));
  zz_142 <= pkg_cat(pkg_toStdLogicVector(zz_72),pkg_toStdLogicVector(pkg_toStdLogic(zz_148 = zz_149)));
  zz_143 <= pkg_stdLogicVector("00");
  zz_144 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_150),pkg_toStdLogicVector(zz_151)) /= pkg_stdLogicVector("00"));
  zz_145 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_152,zz_153) /= pkg_stdLogicVector("000")));
  zz_146 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_154 /= zz_155)),pkg_cat(pkg_toStdLogicVector(zz_156),pkg_cat(zz_157,zz_158)));
  zz_147 <= pkg_stdLogicVector("00000000000000000000000001110000");
  zz_148 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000100000"));
  zz_149 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_150 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001100100")) = pkg_stdLogicVector("00000000000000000000000000100100"));
  zz_151 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000011000001010100")) = pkg_stdLogicVector("00000000000000000001000000010000"));
  zz_152 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_159) = pkg_stdLogicVector("00000000000000000000000001000000")));
  zz_153 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_160 = zz_161)),pkg_toStdLogicVector(pkg_toStdLogic(zz_162 = zz_163)));
  zz_154 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_164 = zz_165)),pkg_toStdLogicVector(pkg_toStdLogic(zz_166 = zz_167)));
  zz_155 <= pkg_stdLogicVector("00");
  zz_156 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_168),pkg_toStdLogicVector(zz_169)) /= pkg_stdLogicVector("00"));
  zz_157 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_170,zz_171) /= pkg_stdLogicVector("00")));
  zz_158 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_172 /= zz_173)),pkg_cat(pkg_toStdLogicVector(zz_174),pkg_cat(zz_175,zz_176)));
  zz_159 <= pkg_stdLogicVector("00000000000000000000000001000100");
  zz_160 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000010100"));
  zz_161 <= pkg_stdLogicVector("00000000000000000010000000010000");
  zz_162 <= (decode_INSTRUCTION and pkg_stdLogicVector("01000000000000000100000000110100"));
  zz_163 <= pkg_stdLogicVector("01000000000000000000000000110000");
  zz_164 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000110100"));
  zz_165 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_166 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001100100"));
  zz_167 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_168 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001010000")) = pkg_stdLogicVector("00000000000000000000000001000000"));
  zz_169 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000011000001000000")) = pkg_stdLogicVector("00000000000000000000000001000000"));
  zz_170 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_177) = pkg_stdLogicVector("00000000000000000001000001010000")));
  zz_171 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_178) = pkg_stdLogicVector("00000000000000000010000001010000")));
  zz_172 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_179) = pkg_stdLogicVector("00000000000000000000000000100000")));
  zz_173 <= pkg_stdLogicVector("0");
  zz_174 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_180),pkg_toStdLogicVector(zz_181)) /= pkg_stdLogicVector("00"));
  zz_175 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_70) /= pkg_stdLogicVector("0")));
  zz_176 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_182 /= zz_183)),pkg_cat(pkg_toStdLogicVector(zz_184),pkg_cat(zz_185,zz_186)));
  zz_177 <= pkg_stdLogicVector("00000000000000000001000001010000");
  zz_178 <= pkg_stdLogicVector("00000000000000000010000001010000");
  zz_179 <= pkg_stdLogicVector("00000000000000000000000000100000");
  zz_180 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000010000")) = pkg_stdLogicVector("00000000000000000010000000000000"));
  zz_181 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000101000000000000")) = pkg_stdLogicVector("00000000000000000001000000000000"));
  zz_182 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000010000")) = pkg_stdLogicVector("00000000000000000000000000010000")));
  zz_183 <= pkg_stdLogicVector("0");
  zz_184 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_187 = zz_188)),pkg_cat(pkg_toStdLogicVector(zz_189),pkg_cat(zz_190,zz_191))) /= pkg_stdLogicVector("0000"));
  zz_185 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_192 = zz_193)) /= pkg_stdLogicVector("0")));
  zz_186 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(zz_194,zz_195) /= pkg_stdLogicVector("000000"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_196 /= zz_197)),pkg_cat(pkg_toStdLogicVector(zz_198),pkg_cat(zz_199,zz_200))));
  zz_187 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000100"));
  zz_188 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_189 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000011000")) = pkg_stdLogicVector("00000000000000000000000000000000"));
  zz_190 <= pkg_toStdLogicVector(zz_69);
  zz_191 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_201) = pkg_stdLogicVector("00000000000000000001000000000000")));
  zz_192 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001011000"));
  zz_193 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_194 <= pkg_toStdLogicVector(zz_71);
  zz_195 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_202 = zz_203)),pkg_cat(pkg_toStdLogicVector(zz_204),pkg_cat(zz_205,zz_206)));
  zz_196 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_207 = zz_208)),pkg_toStdLogicVector(pkg_toStdLogic(zz_209 = zz_210)));
  zz_197 <= pkg_stdLogicVector("00");
  zz_198 <= pkg_toStdLogic(pkg_toStdLogicVector(zz_69) /= pkg_stdLogicVector("0"));
  zz_199 <= pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_211) /= pkg_stdLogicVector("0")));
  zz_200 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_212 /= zz_213)),pkg_cat(pkg_toStdLogicVector(zz_214),pkg_toStdLogicVector(zz_215)));
  zz_201 <= pkg_stdLogicVector("00000000000000000101000000000100");
  zz_202 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000001000000010000"));
  zz_203 <= pkg_stdLogicVector("00000000000000000001000000010000");
  zz_204 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000010000000010000")) = pkg_stdLogicVector("00000000000000000010000000010000"));
  zz_205 <= pkg_toStdLogicVector(zz_70);
  zz_206 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_216 = zz_217)),pkg_toStdLogicVector(pkg_toStdLogic(zz_218 = zz_219)));
  zz_207 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000110000000000100"));
  zz_208 <= pkg_stdLogicVector("00000000000000000110000000000000");
  zz_209 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000101000000000100"));
  zz_210 <= pkg_stdLogicVector("00000000000000000100000000000000");
  zz_211 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000001000000000000")) = pkg_stdLogicVector("00000000000000000001000000000000"));
  zz_212 <= pkg_toStdLogicVector(pkg_toStdLogic((decode_INSTRUCTION and zz_220) = pkg_stdLogicVector("00000000000000000010000000000000")));
  zz_213 <= pkg_stdLogicVector("0");
  zz_214 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_221),pkg_toStdLogicVector(zz_68)) /= pkg_stdLogicVector("00"));
  zz_215 <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_222),pkg_toStdLogicVector(zz_68)) /= pkg_stdLogicVector("00"));
  zz_216 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000001100"));
  zz_217 <= pkg_stdLogicVector("00000000000000000000000000000100");
  zz_218 <= (decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000101000"));
  zz_219 <= pkg_stdLogicVector("00000000000000000000000000000000");
  zz_220 <= pkg_stdLogicVector("00000000000000000011000000000000");
  zz_221 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000010100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  zz_222 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001000100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  process(clk)
  begin
    if rising_edge(clk) then
      if zz_125 = '1' then
        zz_103 <= RegFilePlugin_regFile(to_integer(decode_RegFilePlugin_regFileReadAddress1));
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if zz_126 = '1' then
        zz_104 <= RegFilePlugin_regFile(to_integer(decode_RegFilePlugin_regFileReadAddress2));
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if zz_31 = '1' then
        RegFilePlugin_regFile(to_integer(lastStageRegFileWrite_payload_address)) <= lastStageRegFileWrite_payload_data;
      end if;
    end if;
  end process;

  IBusSimplePlugin_rspJoin_rspBuffer_c : entity work.StreamFifoLowLatency
    port map ( 
      io_push_valid => iBus_rsp_takeWhen_valid,
      io_push_ready => IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready,
      io_push_payload_error => iBus_rsp_takeWhen_payload_error,
      io_push_payload_inst => iBus_rsp_takeWhen_payload_inst,
      io_pop_valid => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid,
      io_pop_ready => IBusSimplePlugin_rspJoin_rspBufferOutput_ready,
      io_pop_payload_error => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error,
      io_pop_payload_inst => IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst,
      io_flush => IBusSimplePlugin_fetcherflushIt,
      io_occupancy => IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy,
      clk => clk,
      reset => reset 
    );
  decode_SRC2 <= zz_87;
  memory_MEMORY_READ_DATA <= dBus_rsp_data;
  decode_CSR_WRITE_OPCODE <= (not ((pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,14,13) = pkg_stdLogicVector("01")) and pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,19,15) = pkg_stdLogicVector("00000"))) or (pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,14,13) = pkg_stdLogicVector("11")) and pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,19,15) = pkg_stdLogicVector("00000")))));
  writeBack_FORMAL_PC_NEXT <= memory_to_writeBack_FORMAL_PC_NEXT;
  memory_FORMAL_PC_NEXT <= execute_to_memory_FORMAL_PC_NEXT;
  execute_FORMAL_PC_NEXT <= decode_to_execute_FORMAL_PC_NEXT;
  decode_FORMAL_PC_NEXT <= (decode_PC + pkg_unsigned("00000000000000000000000000000100"));
  execute_BYPASSABLE_MEMORY_STAGE <= decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  decode_BYPASSABLE_MEMORY_STAGE <= pkg_extract(pkg_extract(zz_67,9,9),0);
  decode_CSR_READ_OPCODE <= pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,13,7) /= pkg_stdLogicVector("0100000"));
  decode_ALU_CTRL <= zz_1;
  zz_2 <= zz_3;
  decode_RS1 <= decode_RegFilePlugin_rs1Data;
  decode_BYPASSABLE_EXECUTE_STAGE <= pkg_extract(pkg_extract(zz_67,10,10),0);
  decode_ALU_BITWISE_CTRL <= zz_4;
  zz_5 <= zz_6;
  decode_SHIFT_CTRL <= zz_7;
  zz_8 <= zz_9;
  decode_MEMORY_ENABLE <= pkg_extract(pkg_extract(zz_67,7,7),0);
  decode_SRC1 <= zz_82;
  writeBack_REGFILE_WRITE_DATA <= memory_to_writeBack_REGFILE_WRITE_DATA;
  execute_REGFILE_WRITE_DATA <= zz_81;
  decode_MEMORY_STORE <= pkg_extract(pkg_extract(zz_67,12,12),0);
  zz_10 <= zz_11;
  zz_12 <= zz_13;
  decode_ENV_CTRL <= zz_14;
  zz_15 <= zz_16;
  decode_SRC_LESS_UNSIGNED <= pkg_extract(pkg_extract(zz_67,11,11),0);
  decode_BRANCH_CTRL <= zz_17;
  zz_18 <= zz_19;
  decode_IS_CSR <= pkg_extract(pkg_extract(zz_67,13,13),0);
  execute_BRANCH_DO <= zz_95;
  decode_SRC2_FORCE_ZERO <= (decode_SRC_ADD_ZERO and (not decode_SRC_USE_SUB_LESS));
  decode_RS2 <= decode_RegFilePlugin_rs2Data;
  memory_MEMORY_ADDRESS_LOW <= execute_to_memory_MEMORY_ADDRESS_LOW;
  execute_MEMORY_ADDRESS_LOW <= pkg_extract(zz_105,1,0);
  execute_BRANCH_CALC <= unsigned(pkg_cat(std_logic_vector(pkg_extract(execute_BranchPlugin_branchAdder,31,1)),std_logic_vector(pkg_unsigned("0"))));
  memory_PC <= execute_to_memory_PC;
  memory_BRANCH_CALC <= execute_to_memory_BRANCH_CALC;
  memory_BRANCH_DO <= execute_to_memory_BRANCH_DO;
  execute_PC <= decode_to_execute_PC;
  execute_RS1 <= decode_to_execute_RS1;
  execute_BRANCH_CTRL <= zz_20;
  decode_RS2_USE <= pkg_extract(pkg_extract(zz_67,15,15),0);
  decode_RS1_USE <= pkg_extract(pkg_extract(zz_67,8,8),0);
  execute_REGFILE_WRITE_VALID <= decode_to_execute_REGFILE_WRITE_VALID;
  execute_BYPASSABLE_EXECUTE_STAGE <= decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  memory_REGFILE_WRITE_VALID <= execute_to_memory_REGFILE_WRITE_VALID;
  memory_INSTRUCTION <= execute_to_memory_INSTRUCTION;
  memory_BYPASSABLE_MEMORY_STAGE <= execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  writeBack_REGFILE_WRITE_VALID <= memory_to_writeBack_REGFILE_WRITE_VALID;
  memory_REGFILE_WRITE_DATA <= execute_to_memory_REGFILE_WRITE_DATA;
  execute_SHIFT_CTRL <= zz_21;
  execute_SRC_LESS_UNSIGNED <= decode_to_execute_SRC_LESS_UNSIGNED;
  execute_SRC2_FORCE_ZERO <= decode_to_execute_SRC2_FORCE_ZERO;
  execute_SRC_USE_SUB_LESS <= decode_to_execute_SRC_USE_SUB_LESS;
  zz_22 <= decode_PC;
  zz_23 <= decode_RS2;
  decode_SRC2_CTRL <= zz_24;
  zz_25 <= decode_RS1;
  decode_SRC1_CTRL <= zz_26;
  decode_SRC_USE_SUB_LESS <= pkg_extract(pkg_extract(zz_67,16,16),0);
  decode_SRC_ADD_ZERO <= pkg_extract(pkg_extract(zz_67,17,17),0);
  execute_SRC_ADD_SUB <= execute_SrcPlugin_addSub;
  execute_SRC_LESS <= execute_SrcPlugin_less;
  execute_ALU_CTRL <= zz_27;
  execute_SRC2 <= decode_to_execute_SRC2;
  execute_ALU_BITWISE_CTRL <= zz_28;
  zz_29 <= writeBack_INSTRUCTION;
  zz_30 <= writeBack_REGFILE_WRITE_VALID;
  process(lastStageRegFileWrite_valid)
  begin
    zz_31 <= pkg_toStdLogic(false);
    if lastStageRegFileWrite_valid = '1' then
      zz_31 <= pkg_toStdLogic(true);
    end if;
  end process;

  decode_INSTRUCTION_ANTICIPATED <= pkg_mux(decode_arbitration_isStuck,decode_INSTRUCTION,IBusSimplePlugin_iBusRsp_output_payload_rsp_inst);
  process(zz_67,decode_INSTRUCTION)
  begin
    decode_REGFILE_WRITE_VALID <= pkg_extract(pkg_extract(zz_67,6,6),0);
    if pkg_toStdLogic(pkg_extract(decode_INSTRUCTION,11,7) = pkg_stdLogicVector("00000")) = '1' then
      decode_REGFILE_WRITE_VALID <= pkg_toStdLogic(false);
    end if;
  end process;

  process(execute_REGFILE_WRITE_DATA,zz_107,execute_CsrPlugin_readData,zz_108,zz_88)
  begin
    zz_39 <= execute_REGFILE_WRITE_DATA;
    if zz_107 = '1' then
      zz_39 <= execute_CsrPlugin_readData;
    end if;
    if zz_108 = '1' then
      zz_39 <= zz_88;
    end if;
  end process;

  execute_SRC1 <= decode_to_execute_SRC1;
  execute_CSR_READ_OPCODE <= decode_to_execute_CSR_READ_OPCODE;
  execute_CSR_WRITE_OPCODE <= decode_to_execute_CSR_WRITE_OPCODE;
  execute_IS_CSR <= decode_to_execute_IS_CSR;
  memory_ENV_CTRL <= zz_40;
  execute_ENV_CTRL <= zz_41;
  writeBack_ENV_CTRL <= zz_42;
  writeBack_MEMORY_STORE <= memory_to_writeBack_MEMORY_STORE;
  process(writeBack_REGFILE_WRITE_DATA,writeBack_arbitration_isValid,writeBack_MEMORY_ENABLE,writeBack_DBusSimplePlugin_rspFormated)
  begin
    zz_43 <= writeBack_REGFILE_WRITE_DATA;
    if (writeBack_arbitration_isValid and writeBack_MEMORY_ENABLE) = '1' then
      zz_43 <= writeBack_DBusSimplePlugin_rspFormated;
    end if;
  end process;

  writeBack_MEMORY_ENABLE <= memory_to_writeBack_MEMORY_ENABLE;
  writeBack_MEMORY_ADDRESS_LOW <= memory_to_writeBack_MEMORY_ADDRESS_LOW;
  writeBack_MEMORY_READ_DATA <= memory_to_writeBack_MEMORY_READ_DATA;
  memory_MEMORY_STORE <= execute_to_memory_MEMORY_STORE;
  memory_MEMORY_ENABLE <= execute_to_memory_MEMORY_ENABLE;
  execute_SRC_ADD <= execute_SrcPlugin_addSub;
  execute_RS2 <= decode_to_execute_RS2;
  execute_INSTRUCTION <= decode_to_execute_INSTRUCTION;
  execute_MEMORY_STORE <= decode_to_execute_MEMORY_STORE;
  execute_MEMORY_ENABLE <= decode_to_execute_MEMORY_ENABLE;
  execute_ALIGNEMENT_FAULT <= pkg_toStdLogic(false);
  process(memory_FORMAL_PC_NEXT,BranchPlugin_jumpInterface_valid,BranchPlugin_jumpInterface_payload)
  begin
    zz_44 <= memory_FORMAL_PC_NEXT;
    if BranchPlugin_jumpInterface_valid = '1' then
      zz_44 <= BranchPlugin_jumpInterface_payload;
    end if;
  end process;

  decode_PC <= IBusSimplePlugin_injector_decodeInput_payload_pc;
  decode_INSTRUCTION <= IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  writeBack_PC <= memory_to_writeBack_PC;
  writeBack_INSTRUCTION <= memory_to_writeBack_INSTRUCTION;
  decode_arbitration_haltItself <= pkg_toStdLogic(false);
  process(CsrPlugin_interrupt_valid,CsrPlugin_allowInterrupts,decode_arbitration_isValid,writeBack_arbitration_isValid,writeBack_ENV_CTRL,memory_arbitration_isValid,memory_ENV_CTRL,execute_arbitration_isValid,execute_ENV_CTRL,zz_89,zz_90)
  begin
    decode_arbitration_haltByOther <= pkg_toStdLogic(false);
    if (CsrPlugin_interrupt_valid and CsrPlugin_allowInterrupts) = '1' then
      decode_arbitration_haltByOther <= decode_arbitration_isValid;
    end if;
    if pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector((writeBack_arbitration_isValid and pkg_toStdLogic(writeBack_ENV_CTRL = EnvCtrlEnum_defaultEncoding_XRET))),pkg_cat(pkg_toStdLogicVector((memory_arbitration_isValid and pkg_toStdLogic(memory_ENV_CTRL = EnvCtrlEnum_defaultEncoding_XRET))),pkg_toStdLogicVector((execute_arbitration_isValid and pkg_toStdLogic(execute_ENV_CTRL = EnvCtrlEnum_defaultEncoding_XRET))))) /= pkg_stdLogicVector("000")) = '1' then
      decode_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
    if (decode_arbitration_isValid and (zz_89 or zz_90)) = '1' then
      decode_arbitration_haltByOther <= pkg_toStdLogic(true);
    end if;
  end process;

  process(decode_arbitration_isFlushed)
  begin
    decode_arbitration_removeIt <= pkg_toStdLogic(false);
    if decode_arbitration_isFlushed = '1' then
      decode_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  decode_arbitration_flushIt <= pkg_toStdLogic(false);
  decode_arbitration_flushNext <= pkg_toStdLogic(false);
  process(execute_arbitration_isValid,execute_MEMORY_ENABLE,dBus_cmd_ready,execute_DBusSimplePlugin_skipCmd,zz_57,zz_107,execute_CsrPlugin_blockedBySideEffects,zz_108,zz_109,execute_LightShifterPlugin_done)
  begin
    execute_arbitration_haltItself <= pkg_toStdLogic(false);
    if ((((execute_arbitration_isValid and execute_MEMORY_ENABLE) and (not dBus_cmd_ready)) and (not execute_DBusSimplePlugin_skipCmd)) and (not zz_57)) = '1' then
      execute_arbitration_haltItself <= pkg_toStdLogic(true);
    end if;
    if zz_107 = '1' then
      if execute_CsrPlugin_blockedBySideEffects = '1' then
        execute_arbitration_haltItself <= pkg_toStdLogic(true);
      end if;
    end if;
    if zz_108 = '1' then
      if zz_109 = '1' then
        if execute_LightShifterPlugin_done = '0' then
          execute_arbitration_haltItself <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
  end process;

  execute_arbitration_haltByOther <= pkg_toStdLogic(false);
  process(execute_arbitration_isFlushed)
  begin
    execute_arbitration_removeIt <= pkg_toStdLogic(false);
    if execute_arbitration_isFlushed = '1' then
      execute_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  execute_arbitration_flushIt <= pkg_toStdLogic(false);
  execute_arbitration_flushNext <= pkg_toStdLogic(false);
  process(memory_arbitration_isValid,memory_MEMORY_ENABLE,memory_MEMORY_STORE,dBus_rsp_ready)
  begin
    memory_arbitration_haltItself <= pkg_toStdLogic(false);
    if (((memory_arbitration_isValid and memory_MEMORY_ENABLE) and (not memory_MEMORY_STORE)) and ((not dBus_rsp_ready) or pkg_toStdLogic(false))) = '1' then
      memory_arbitration_haltItself <= pkg_toStdLogic(true);
    end if;
  end process;

  memory_arbitration_haltByOther <= pkg_toStdLogic(false);
  process(memory_arbitration_isFlushed)
  begin
    memory_arbitration_removeIt <= pkg_toStdLogic(false);
    if memory_arbitration_isFlushed = '1' then
      memory_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  memory_arbitration_flushIt <= pkg_toStdLogic(false);
  process(BranchPlugin_jumpInterface_valid)
  begin
    memory_arbitration_flushNext <= pkg_toStdLogic(false);
    if BranchPlugin_jumpInterface_valid = '1' then
      memory_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  writeBack_arbitration_haltItself <= pkg_toStdLogic(false);
  writeBack_arbitration_haltByOther <= pkg_toStdLogic(false);
  process(writeBack_arbitration_isFlushed)
  begin
    writeBack_arbitration_removeIt <= pkg_toStdLogic(false);
    if writeBack_arbitration_isFlushed = '1' then
      writeBack_arbitration_removeIt <= pkg_toStdLogic(true);
    end if;
  end process;

  writeBack_arbitration_flushIt <= pkg_toStdLogic(false);
  process(zz_110,zz_111)
  begin
    writeBack_arbitration_flushNext <= pkg_toStdLogic(false);
    if zz_110 = '1' then
      writeBack_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
    if zz_111 = '1' then
      writeBack_arbitration_flushNext <= pkg_toStdLogic(true);
    end if;
  end process;

  lastStageInstruction <= writeBack_INSTRUCTION;
  lastStagePc <= writeBack_PC;
  lastStageIsValid <= writeBack_arbitration_isValid;
  lastStageIsFiring <= writeBack_arbitration_isFiring;
  process(zz_110,zz_111)
  begin
    IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(false);
    if zz_110 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
    if zz_111 = '1' then
      IBusSimplePlugin_fetcherHalt <= pkg_toStdLogic(true);
    end if;
  end process;

  process(writeBack_arbitration_flushNext,memory_arbitration_flushNext,execute_arbitration_flushNext,decode_arbitration_flushNext)
  begin
    IBusSimplePlugin_fetcherflushIt <= pkg_toStdLogic(false);
    if pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(execute_arbitration_flushNext),pkg_toStdLogicVector(decode_arbitration_flushNext)))) /= pkg_stdLogicVector("0000")) = '1' then
      IBusSimplePlugin_fetcherflushIt <= pkg_toStdLogic(true);
    end if;
  end process;

  process(IBusSimplePlugin_iBusRsp_stages_1_input_valid,IBusSimplePlugin_injector_decodeInput_valid)
  begin
    IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(false);
    if IBusSimplePlugin_iBusRsp_stages_1_input_valid = '1' then
      IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(true);
    end if;
    if IBusSimplePlugin_injector_decodeInput_valid = '1' then
      IBusSimplePlugin_incomingInstruction <= pkg_toStdLogic(true);
    end if;
  end process;

  CsrPlugin_inWfi <= pkg_toStdLogic(false);
  CsrPlugin_thirdPartyWake <= pkg_toStdLogic(false);
  process(zz_110,zz_111)
  begin
    CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(false);
    if zz_110 = '1' then
      CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(true);
    end if;
    if zz_111 = '1' then
      CsrPlugin_jumpInterface_valid <= pkg_toStdLogic(true);
    end if;
  end process;

  process(zz_110,CsrPlugin_xtvec_base,zz_111,zz_112,CsrPlugin_mepc)
  begin
    CsrPlugin_jumpInterface_payload <= pkg_unsigned("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    if zz_110 = '1' then
      CsrPlugin_jumpInterface_payload <= unsigned(pkg_cat(std_logic_vector(CsrPlugin_xtvec_base),std_logic_vector(pkg_unsigned("00"))));
    end if;
    if zz_111 = '1' then
      case zz_112 is
        when "11" =>
          CsrPlugin_jumpInterface_payload <= CsrPlugin_mepc;
        when others =>
      end case;
    end if;
  end process;

  CsrPlugin_forceMachineWire <= pkg_toStdLogic(false);
  CsrPlugin_allowInterrupts <= pkg_toStdLogic(true);
  CsrPlugin_allowException <= pkg_toStdLogic(true);
  IBusSimplePlugin_jump_pcLoad_valid <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(BranchPlugin_jumpInterface_valid),pkg_toStdLogicVector(CsrPlugin_jumpInterface_valid)) /= pkg_stdLogicVector("00"));
  zz_45 <= unsigned(pkg_cat(pkg_toStdLogicVector(BranchPlugin_jumpInterface_valid),pkg_toStdLogicVector(CsrPlugin_jumpInterface_valid)));
  IBusSimplePlugin_jump_pcLoad_payload <= pkg_mux(pkg_extract(std_logic_vector((zz_45 and pkg_not((zz_45 - pkg_unsigned("01"))))),0),CsrPlugin_jumpInterface_payload,BranchPlugin_jumpInterface_payload);
  process(IBusSimplePlugin_jump_pcLoad_valid)
  begin
    IBusSimplePlugin_fetchPc_corrected <= pkg_toStdLogic(false);
    if IBusSimplePlugin_jump_pcLoad_valid = '1' then
      IBusSimplePlugin_fetchPc_corrected <= pkg_toStdLogic(true);
    end if;
  end process;

  process(IBusSimplePlugin_iBusRsp_stages_1_input_ready)
  begin
    IBusSimplePlugin_fetchPc_pcRegPropagate <= pkg_toStdLogic(false);
    if IBusSimplePlugin_iBusRsp_stages_1_input_ready = '1' then
      IBusSimplePlugin_fetchPc_pcRegPropagate <= pkg_toStdLogic(true);
    end if;
  end process;

  process(IBusSimplePlugin_fetchPc_pcReg,IBusSimplePlugin_fetchPc_inc,IBusSimplePlugin_jump_pcLoad_valid,IBusSimplePlugin_jump_pcLoad_payload)
  begin
    IBusSimplePlugin_fetchPc_pc <= (IBusSimplePlugin_fetchPc_pcReg + pkg_resize(unsigned(pkg_cat(pkg_toStdLogicVector(IBusSimplePlugin_fetchPc_inc),pkg_stdLogicVector("00"))),32));
    if IBusSimplePlugin_jump_pcLoad_valid = '1' then
      IBusSimplePlugin_fetchPc_pc <= IBusSimplePlugin_jump_pcLoad_payload;
    end if;
    IBusSimplePlugin_fetchPc_pc(0) <= pkg_toStdLogic(false);
    IBusSimplePlugin_fetchPc_pc(1) <= pkg_toStdLogic(false);
  end process;

  IBusSimplePlugin_fetchPc_output_valid <= ((not IBusSimplePlugin_fetcherHalt) and IBusSimplePlugin_fetchPc_booted);
  IBusSimplePlugin_fetchPc_output_payload <= IBusSimplePlugin_fetchPc_pc;
  IBusSimplePlugin_iBusRsp_stages_0_input_valid <= IBusSimplePlugin_fetchPc_output_valid;
  IBusSimplePlugin_fetchPc_output_ready <= IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  IBusSimplePlugin_iBusRsp_stages_0_input_payload <= IBusSimplePlugin_fetchPc_output_payload;
  IBusSimplePlugin_iBusRsp_stages_0_inputSample <= pkg_toStdLogic(true);
  process(IBusSimplePlugin_iBusRsp_stages_0_input_valid,IBusSimplePlugin_cmd_valid,IBusSimplePlugin_cmd_ready)
  begin
    IBusSimplePlugin_iBusRsp_stages_0_halt <= pkg_toStdLogic(false);
    if (IBusSimplePlugin_iBusRsp_stages_0_input_valid and ((not IBusSimplePlugin_cmd_valid) or (not IBusSimplePlugin_cmd_ready))) = '1' then
      IBusSimplePlugin_iBusRsp_stages_0_halt <= pkg_toStdLogic(true);
    end if;
  end process;

  zz_46 <= (not IBusSimplePlugin_iBusRsp_stages_0_halt);
  IBusSimplePlugin_iBusRsp_stages_0_input_ready <= (IBusSimplePlugin_iBusRsp_stages_0_output_ready and zz_46);
  IBusSimplePlugin_iBusRsp_stages_0_output_valid <= (IBusSimplePlugin_iBusRsp_stages_0_input_valid and zz_46);
  IBusSimplePlugin_iBusRsp_stages_0_output_payload <= IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  IBusSimplePlugin_iBusRsp_stages_1_halt <= pkg_toStdLogic(false);
  zz_47 <= (not IBusSimplePlugin_iBusRsp_stages_1_halt);
  IBusSimplePlugin_iBusRsp_stages_1_input_ready <= (IBusSimplePlugin_iBusRsp_stages_1_output_ready and zz_47);
  IBusSimplePlugin_iBusRsp_stages_1_output_valid <= (IBusSimplePlugin_iBusRsp_stages_1_input_valid and zz_47);
  IBusSimplePlugin_iBusRsp_stages_1_output_payload <= IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  IBusSimplePlugin_iBusRsp_stages_0_output_ready <= zz_48;
  zz_48 <= ((pkg_toStdLogic(false) and (not zz_49)) or IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  zz_49 <= zz_50;
  IBusSimplePlugin_iBusRsp_stages_1_input_valid <= zz_49;
  IBusSimplePlugin_iBusRsp_stages_1_input_payload <= IBusSimplePlugin_fetchPc_pcReg;
  process(IBusSimplePlugin_injector_decodeInput_valid,IBusSimplePlugin_pcValids_0)
  begin
    IBusSimplePlugin_iBusRsp_readyForError <= pkg_toStdLogic(true);
    if IBusSimplePlugin_injector_decodeInput_valid = '1' then
      IBusSimplePlugin_iBusRsp_readyForError <= pkg_toStdLogic(false);
    end if;
    if (not IBusSimplePlugin_pcValids_0) = '1' then
      IBusSimplePlugin_iBusRsp_readyForError <= pkg_toStdLogic(false);
    end if;
  end process;

  IBusSimplePlugin_iBusRsp_output_ready <= ((pkg_toStdLogic(false) and (not IBusSimplePlugin_injector_decodeInput_valid)) or IBusSimplePlugin_injector_decodeInput_ready);
  IBusSimplePlugin_injector_decodeInput_valid <= zz_51;
  IBusSimplePlugin_injector_decodeInput_payload_pc <= zz_52;
  IBusSimplePlugin_injector_decodeInput_payload_rsp_error <= zz_53;
  IBusSimplePlugin_injector_decodeInput_payload_rsp_inst <= zz_54;
  IBusSimplePlugin_injector_decodeInput_payload_isRvc <= zz_55;
  IBusSimplePlugin_pcValids_0 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
  IBusSimplePlugin_pcValids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
  IBusSimplePlugin_pcValids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
  IBusSimplePlugin_pcValids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_4;
  IBusSimplePlugin_injector_decodeInput_ready <= (not decode_arbitration_isStuck);
  decode_arbitration_isValid <= (IBusSimplePlugin_injector_decodeInput_valid and (not IBusSimplePlugin_injector_decodeRemoved));
  iBus_cmd_valid <= IBusSimplePlugin_cmd_valid;
  IBusSimplePlugin_cmd_ready <= iBus_cmd_ready;
  iBus_cmd_payload_pc <= IBusSimplePlugin_cmd_payload_pc;
  IBusSimplePlugin_pendingCmdNext <= ((IBusSimplePlugin_pendingCmd + pkg_resize(unsigned(pkg_toStdLogicVector((IBusSimplePlugin_cmd_valid and IBusSimplePlugin_cmd_ready))),3)) - pkg_resize(unsigned(pkg_toStdLogicVector(iBus_rsp_valid)),3));
  IBusSimplePlugin_cmd_valid <= ((IBusSimplePlugin_iBusRsp_stages_0_input_valid and IBusSimplePlugin_iBusRsp_stages_0_output_ready) and pkg_toStdLogic(IBusSimplePlugin_pendingCmd /= pkg_unsigned("111")));
  IBusSimplePlugin_cmd_payload_pc <= unsigned(pkg_cat(std_logic_vector(pkg_extract(IBusSimplePlugin_iBusRsp_stages_0_input_payload,31,2)),std_logic_vector(pkg_unsigned("00"))));
  iBus_rsp_takeWhen_valid <= (iBus_rsp_valid and (not pkg_toStdLogic(IBusSimplePlugin_rspJoin_discardCounter /= pkg_unsigned("000"))));
  iBus_rsp_takeWhen_payload_error <= iBus_rsp_payload_error;
  iBus_rsp_takeWhen_payload_inst <= iBus_rsp_payload_inst;
  IBusSimplePlugin_rspJoin_rspBufferOutput_valid <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst <= IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  IBusSimplePlugin_rspJoin_fetchRsp_pc <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  process(IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error,IBusSimplePlugin_rspJoin_rspBufferOutput_valid)
  begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error <= IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if (not IBusSimplePlugin_rspJoin_rspBufferOutput_valid) = '1' then
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error <= pkg_toStdLogic(false);
    end if;
  end process;

  IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst <= IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  IBusSimplePlugin_rspJoin_exceptionDetected <= pkg_toStdLogic(false);
  IBusSimplePlugin_rspJoin_redoRequired <= pkg_toStdLogic(false);
  IBusSimplePlugin_rspJoin_join_valid <= (IBusSimplePlugin_iBusRsp_stages_1_output_valid and IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  IBusSimplePlugin_rspJoin_join_payload_pc <= IBusSimplePlugin_rspJoin_fetchRsp_pc;
  IBusSimplePlugin_rspJoin_join_payload_rsp_error <= IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  IBusSimplePlugin_rspJoin_join_payload_rsp_inst <= IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  IBusSimplePlugin_rspJoin_join_payload_isRvc <= IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  IBusSimplePlugin_iBusRsp_stages_1_output_ready <= pkg_mux(IBusSimplePlugin_iBusRsp_stages_1_output_valid,(IBusSimplePlugin_rspJoin_join_valid and IBusSimplePlugin_rspJoin_join_ready),IBusSimplePlugin_rspJoin_join_ready);
  IBusSimplePlugin_rspJoin_rspBufferOutput_ready <= (IBusSimplePlugin_rspJoin_join_valid and IBusSimplePlugin_rspJoin_join_ready);
  zz_56 <= (not (IBusSimplePlugin_rspJoin_exceptionDetected or IBusSimplePlugin_rspJoin_redoRequired));
  IBusSimplePlugin_rspJoin_join_ready <= (IBusSimplePlugin_iBusRsp_output_ready and zz_56);
  IBusSimplePlugin_iBusRsp_output_valid <= (IBusSimplePlugin_rspJoin_join_valid and zz_56);
  IBusSimplePlugin_iBusRsp_output_payload_pc <= IBusSimplePlugin_rspJoin_join_payload_pc;
  IBusSimplePlugin_iBusRsp_output_payload_rsp_error <= IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  IBusSimplePlugin_iBusRsp_output_payload_rsp_inst <= IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  IBusSimplePlugin_iBusRsp_output_payload_isRvc <= IBusSimplePlugin_rspJoin_join_payload_isRvc;
  zz_57 <= pkg_toStdLogic(false);
  process(execute_ALIGNEMENT_FAULT)
  begin
    execute_DBusSimplePlugin_skipCmd <= pkg_toStdLogic(false);
    if execute_ALIGNEMENT_FAULT = '1' then
      execute_DBusSimplePlugin_skipCmd <= pkg_toStdLogic(true);
    end if;
  end process;

  dBus_cmd_valid <= (((((execute_arbitration_isValid and execute_MEMORY_ENABLE) and (not execute_arbitration_isStuckByOthers)) and (not execute_arbitration_isFlushed)) and (not execute_DBusSimplePlugin_skipCmd)) and (not zz_57));
  dBus_cmd_payload_wr <= execute_MEMORY_STORE;
  zz_106 <= unsigned(pkg_extract(execute_INSTRUCTION,13,12));
  process(zz_106,execute_RS2)
  begin
    case zz_106 is
      when "00" =>
        zz_58 <= pkg_cat(pkg_cat(pkg_cat(pkg_extract(execute_RS2,7,0),pkg_extract(execute_RS2,7,0)),pkg_extract(execute_RS2,7,0)),pkg_extract(execute_RS2,7,0));
      when "01" =>
        zz_58 <= pkg_cat(pkg_extract(execute_RS2,15,0),pkg_extract(execute_RS2,15,0));
      when others =>
        zz_58 <= pkg_extract(execute_RS2,31,0);
    end case;
  end process;

  dBus_cmd_payload_data <= zz_58;
  process(zz_106)
  begin
    case zz_106 is
      when "00" =>
        zz_59 <= pkg_stdLogicVector("0001");
      when "01" =>
        zz_59 <= pkg_stdLogicVector("0011");
      when others =>
        zz_59 <= pkg_stdLogicVector("1111");
    end case;
  end process;

  execute_DBusSimplePlugin_formalMask <= std_logic_vector(shift_left(unsigned(zz_59),to_integer(pkg_extract(zz_105,1,0))));
  zz_105 <= unsigned(execute_SRC_ADD);
  process(writeBack_MEMORY_READ_DATA,writeBack_MEMORY_ADDRESS_LOW)
  begin
    writeBack_DBusSimplePlugin_rspShifted <= writeBack_MEMORY_READ_DATA;
    case writeBack_MEMORY_ADDRESS_LOW is
      when "01" =>
        writeBack_DBusSimplePlugin_rspShifted(7 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,15,8);
      when "10" =>
        writeBack_DBusSimplePlugin_rspShifted(15 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,31,16);
      when "11" =>
        writeBack_DBusSimplePlugin_rspShifted(7 downto 0) <= pkg_extract(writeBack_MEMORY_READ_DATA,31,24);
      when others =>
    end case;
  end process;

  zz_60 <= (pkg_extract(writeBack_DBusSimplePlugin_rspShifted,7) and (not pkg_extract(writeBack_INSTRUCTION,14)));
  process(zz_60,writeBack_DBusSimplePlugin_rspShifted)
  begin
    zz_61(31) <= zz_60;
    zz_61(30) <= zz_60;
    zz_61(29) <= zz_60;
    zz_61(28) <= zz_60;
    zz_61(27) <= zz_60;
    zz_61(26) <= zz_60;
    zz_61(25) <= zz_60;
    zz_61(24) <= zz_60;
    zz_61(23) <= zz_60;
    zz_61(22) <= zz_60;
    zz_61(21) <= zz_60;
    zz_61(20) <= zz_60;
    zz_61(19) <= zz_60;
    zz_61(18) <= zz_60;
    zz_61(17) <= zz_60;
    zz_61(16) <= zz_60;
    zz_61(15) <= zz_60;
    zz_61(14) <= zz_60;
    zz_61(13) <= zz_60;
    zz_61(12) <= zz_60;
    zz_61(11) <= zz_60;
    zz_61(10) <= zz_60;
    zz_61(9) <= zz_60;
    zz_61(8) <= zz_60;
    zz_61(7 downto 0) <= pkg_extract(writeBack_DBusSimplePlugin_rspShifted,7,0);
  end process;

  zz_62 <= (pkg_extract(writeBack_DBusSimplePlugin_rspShifted,15) and (not pkg_extract(writeBack_INSTRUCTION,14)));
  process(zz_62,writeBack_DBusSimplePlugin_rspShifted)
  begin
    zz_63(31) <= zz_62;
    zz_63(30) <= zz_62;
    zz_63(29) <= zz_62;
    zz_63(28) <= zz_62;
    zz_63(27) <= zz_62;
    zz_63(26) <= zz_62;
    zz_63(25) <= zz_62;
    zz_63(24) <= zz_62;
    zz_63(23) <= zz_62;
    zz_63(22) <= zz_62;
    zz_63(21) <= zz_62;
    zz_63(20) <= zz_62;
    zz_63(19) <= zz_62;
    zz_63(18) <= zz_62;
    zz_63(17) <= zz_62;
    zz_63(16) <= zz_62;
    zz_63(15 downto 0) <= pkg_extract(writeBack_DBusSimplePlugin_rspShifted,15,0);
  end process;

  process(zz_123,zz_61,zz_63,writeBack_DBusSimplePlugin_rspShifted)
  begin
    case zz_123 is
      when "00" =>
        writeBack_DBusSimplePlugin_rspFormated <= zz_61;
      when "01" =>
        writeBack_DBusSimplePlugin_rspFormated <= zz_63;
      when others =>
        writeBack_DBusSimplePlugin_rspFormated <= writeBack_DBusSimplePlugin_rspShifted;
    end case;
  end process;

  process(CsrPlugin_forceMachineWire)
  begin
    CsrPlugin_privilege <= pkg_unsigned("11");
    if CsrPlugin_forceMachineWire = '1' then
      CsrPlugin_privilege <= pkg_unsigned("11");
    end if;
  end process;

  CsrPlugin_misa_base <= pkg_unsigned("01");
  CsrPlugin_misa_extensions <= pkg_stdLogicVector("00000000000000000001000010");
  CsrPlugin_mtvec_mode <= pkg_stdLogicVector("00");
  CsrPlugin_mtvec_base <= pkg_unsigned("000000000000000000000000001000");
  zz_64 <= (CsrPlugin_mip_MTIP and CsrPlugin_mie_MTIE);
  zz_65 <= (CsrPlugin_mip_MSIP and CsrPlugin_mie_MSIE);
  zz_66 <= (CsrPlugin_mip_MEIP and CsrPlugin_mie_MEIE);
  CsrPlugin_exception <= pkg_toStdLogic(false);
  CsrPlugin_lastStageWasWfi <= pkg_toStdLogic(false);
  process(writeBack_arbitration_isValid,memory_arbitration_isValid,execute_arbitration_isValid,IBusSimplePlugin_pcValids_0,CsrPlugin_hadException)
  begin
    CsrPlugin_pipelineLiberator_done <= ((not pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_isValid),pkg_cat(pkg_toStdLogicVector(memory_arbitration_isValid),pkg_toStdLogicVector(execute_arbitration_isValid))) /= pkg_stdLogicVector("000"))) and IBusSimplePlugin_pcValids_0);
    if CsrPlugin_hadException = '1' then
      CsrPlugin_pipelineLiberator_done <= pkg_toStdLogic(false);
    end if;
  end process;

  CsrPlugin_interruptJump <= ((CsrPlugin_interrupt_valid and CsrPlugin_pipelineLiberator_done) and CsrPlugin_allowInterrupts);
  CsrPlugin_targetPrivilege <= CsrPlugin_interrupt_targetPrivilege;
  CsrPlugin_trapCause <= CsrPlugin_interrupt_code;
  process(CsrPlugin_targetPrivilege,CsrPlugin_mtvec_mode)
  begin
    CsrPlugin_xtvec_mode <= pkg_stdLogicVector("XX");
    case CsrPlugin_targetPrivilege is
      when "11" =>
        CsrPlugin_xtvec_mode <= CsrPlugin_mtvec_mode;
      when others =>
    end case;
  end process;

  process(CsrPlugin_targetPrivilege,CsrPlugin_mtvec_base)
  begin
    CsrPlugin_xtvec_base <= pkg_unsigned("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    case CsrPlugin_targetPrivilege is
      when "11" =>
        CsrPlugin_xtvec_base <= CsrPlugin_mtvec_base;
      when others =>
    end case;
  end process;

  contextSwitching <= CsrPlugin_jumpInterface_valid;
  execute_CsrPlugin_blockedBySideEffects <= pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_isValid),pkg_toStdLogicVector(memory_arbitration_isValid)) /= pkg_stdLogicVector("00"));
  process(execute_CsrPlugin_csrAddress,execute_CSR_READ_OPCODE,CsrPlugin_privilege,execute_arbitration_isValid,execute_IS_CSR)
  begin
    execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(true);
    case execute_CsrPlugin_csrAddress is
      when "001100000000" =>
        execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
      when "001101000100" =>
        execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
      when "001100000100" =>
        execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
      when "001101000010" =>
        if execute_CSR_READ_OPCODE = '1' then
          execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
        end if;
      when others =>
    end case;
    if pkg_toStdLogic(CsrPlugin_privilege < unsigned(pkg_extract(execute_CsrPlugin_csrAddress,9,8))) = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(true);
    end if;
    if ((not execute_arbitration_isValid) or (not execute_IS_CSR)) = '1' then
      execute_CsrPlugin_illegalAccess <= pkg_toStdLogic(false);
    end if;
  end process;

  process(execute_arbitration_isValid,execute_ENV_CTRL,CsrPlugin_privilege,execute_INSTRUCTION)
  begin
    execute_CsrPlugin_illegalInstruction <= pkg_toStdLogic(false);
    if (execute_arbitration_isValid and pkg_toStdLogic(execute_ENV_CTRL = EnvCtrlEnum_defaultEncoding_XRET)) = '1' then
      if pkg_toStdLogic(CsrPlugin_privilege < unsigned(pkg_extract(execute_INSTRUCTION,29,28))) = '1' then
        execute_CsrPlugin_illegalInstruction <= pkg_toStdLogic(true);
      end if;
    end if;
  end process;

  process(execute_CsrPlugin_csrAddress,CsrPlugin_mstatus_MPP,CsrPlugin_mstatus_MPIE,CsrPlugin_mstatus_MIE,CsrPlugin_mip_MEIP,CsrPlugin_mip_MTIP,CsrPlugin_mip_MSIP,CsrPlugin_mie_MEIE,CsrPlugin_mie_MTIE,CsrPlugin_mie_MSIE,CsrPlugin_mcause_interrupt,CsrPlugin_mcause_exceptionCode)
  begin
    execute_CsrPlugin_readData <= pkg_stdLogicVector("00000000000000000000000000000000");
    case execute_CsrPlugin_csrAddress is
      when "001100000000" =>
        execute_CsrPlugin_readData(12 downto 11) <= std_logic_vector(CsrPlugin_mstatus_MPP);
        execute_CsrPlugin_readData(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mstatus_MPIE);
        execute_CsrPlugin_readData(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mstatus_MIE);
      when "001101000100" =>
        execute_CsrPlugin_readData(11 downto 11) <= pkg_toStdLogicVector(CsrPlugin_mip_MEIP);
        execute_CsrPlugin_readData(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mip_MTIP);
        execute_CsrPlugin_readData(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mip_MSIP);
      when "001100000100" =>
        execute_CsrPlugin_readData(11 downto 11) <= pkg_toStdLogicVector(CsrPlugin_mie_MEIE);
        execute_CsrPlugin_readData(7 downto 7) <= pkg_toStdLogicVector(CsrPlugin_mie_MTIE);
        execute_CsrPlugin_readData(3 downto 3) <= pkg_toStdLogicVector(CsrPlugin_mie_MSIE);
      when "001101000010" =>
        execute_CsrPlugin_readData(31 downto 31) <= pkg_toStdLogicVector(CsrPlugin_mcause_interrupt);
        execute_CsrPlugin_readData(3 downto 0) <= std_logic_vector(CsrPlugin_mcause_exceptionCode);
      when others =>
    end case;
  end process;

  execute_CsrPlugin_writeInstruction <= ((execute_arbitration_isValid and execute_IS_CSR) and execute_CSR_WRITE_OPCODE);
  execute_CsrPlugin_readInstruction <= ((execute_arbitration_isValid and execute_IS_CSR) and execute_CSR_READ_OPCODE);
  execute_CsrPlugin_writeEnable <= ((execute_CsrPlugin_writeInstruction and (not execute_CsrPlugin_blockedBySideEffects)) and (not execute_arbitration_isStuckByOthers));
  execute_CsrPlugin_readEnable <= ((execute_CsrPlugin_readInstruction and (not execute_CsrPlugin_blockedBySideEffects)) and (not execute_arbitration_isStuckByOthers));
  execute_CsrPlugin_readToWriteData <= execute_CsrPlugin_readData;
  process(zz_124,execute_SRC1,execute_INSTRUCTION,execute_CsrPlugin_readToWriteData)
  begin
    case zz_124 is
      when '0' =>
        execute_CsrPlugin_writeData <= execute_SRC1;
      when others =>
        execute_CsrPlugin_writeData <= pkg_mux(pkg_extract(execute_INSTRUCTION,12),(execute_CsrPlugin_readToWriteData and pkg_not(execute_SRC1)),(execute_CsrPlugin_readToWriteData or execute_SRC1));
    end case;
  end process;

  execute_CsrPlugin_csrAddress <= pkg_extract(execute_INSTRUCTION,31,20);
  zz_68 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000100000001010000")) = pkg_stdLogicVector("00000000000000000100000001010000"));
  zz_69 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000110000000000100")) = pkg_stdLogicVector("00000000000000000010000000000000"));
  zz_70 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001010000")) = pkg_stdLogicVector("00000000000000000000000000010000"));
  zz_71 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000001001000")) = pkg_stdLogicVector("00000000000000000000000001001000"));
  zz_72 <= pkg_toStdLogic((decode_INSTRUCTION and pkg_stdLogicVector("00000000000000000000000000000100")) = pkg_stdLogicVector("00000000000000000000000000000100"));
  zz_67 <= pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_71),pkg_toStdLogicVector(pkg_toStdLogic(zz_127 = zz_128))) /= pkg_stdLogicVector("00"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(pkg_toStdLogic(zz_129 = zz_130)) /= pkg_stdLogicVector("0"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(pkg_toStdLogicVector(zz_131) /= pkg_stdLogicVector("0"))),pkg_cat(pkg_toStdLogicVector(pkg_toStdLogic(zz_132 /= zz_133)),pkg_cat(pkg_toStdLogicVector(zz_134),pkg_cat(zz_135,zz_136))))));
  zz_73 <= pkg_extract(zz_67,1,0);
  zz_38 <= zz_73;
  zz_74 <= pkg_extract(zz_67,3,2);
  zz_37 <= zz_74;
  zz_75 <= pkg_extract(zz_67,5,4);
  zz_36 <= zz_75;
  zz_76 <= pkg_extract(zz_67,19,18);
  zz_35 <= zz_76;
  zz_77 <= pkg_extract(zz_67,21,20);
  zz_34 <= zz_77;
  zz_78 <= pkg_extract(zz_67,22,22);
  zz_33 <= zz_78;
  zz_79 <= pkg_extract(zz_67,24,23);
  zz_32 <= zz_79;
  decode_RegFilePlugin_regFileReadAddress1 <= unsigned(pkg_extract(decode_INSTRUCTION_ANTICIPATED,19,15));
  decode_RegFilePlugin_regFileReadAddress2 <= unsigned(pkg_extract(decode_INSTRUCTION_ANTICIPATED,24,20));
  decode_RegFilePlugin_rs1Data <= zz_103;
  decode_RegFilePlugin_rs2Data <= zz_104;
  process(zz_30,writeBack_arbitration_isFiring,zz_80)
  begin
    lastStageRegFileWrite_valid <= (zz_30 and writeBack_arbitration_isFiring);
    if zz_80 = '1' then
      lastStageRegFileWrite_valid <= pkg_toStdLogic(true);
    end if;
  end process;

  lastStageRegFileWrite_payload_address <= unsigned(pkg_extract(zz_29,11,7));
  lastStageRegFileWrite_payload_data <= zz_43;
  process(execute_ALU_BITWISE_CTRL,execute_SRC1,execute_SRC2)
  begin
    case execute_ALU_BITWISE_CTRL is
      when AluBitwiseCtrlEnum_defaultEncoding_AND_1 =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 and execute_SRC2);
      when AluBitwiseCtrlEnum_defaultEncoding_OR_1 =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 or execute_SRC2);
      when others =>
        execute_IntAluPlugin_bitwise <= (execute_SRC1 xor execute_SRC2);
    end case;
  end process;

  process(execute_ALU_CTRL,execute_IntAluPlugin_bitwise,execute_SRC_LESS,execute_SRC_ADD_SUB)
  begin
    case execute_ALU_CTRL is
      when AluCtrlEnum_defaultEncoding_BITWISE =>
        zz_81 <= execute_IntAluPlugin_bitwise;
      when AluCtrlEnum_defaultEncoding_SLT_SLTU =>
        zz_81 <= pkg_resize(pkg_toStdLogicVector(execute_SRC_LESS),32);
      when others =>
        zz_81 <= execute_SRC_ADD_SUB;
    end case;
  end process;

  process(decode_SRC1_CTRL,zz_25,decode_INSTRUCTION)
  begin
    case decode_SRC1_CTRL is
      when Src1CtrlEnum_defaultEncoding_RS =>
        zz_82 <= zz_25;
      when Src1CtrlEnum_defaultEncoding_PC_INCREMENT =>
        zz_82 <= pkg_resize(pkg_stdLogicVector("100"),32);
      when Src1CtrlEnum_defaultEncoding_IMU =>
        zz_82 <= pkg_cat(pkg_extract(decode_INSTRUCTION,31,12),std_logic_vector(pkg_unsigned("000000000000")));
      when others =>
        zz_82 <= pkg_resize(pkg_extract(decode_INSTRUCTION,19,15),32);
    end case;
  end process;

  zz_83 <= pkg_extract(pkg_extract(decode_INSTRUCTION,31,20),11);
  process(zz_83)
  begin
    zz_84(19) <= zz_83;
    zz_84(18) <= zz_83;
    zz_84(17) <= zz_83;
    zz_84(16) <= zz_83;
    zz_84(15) <= zz_83;
    zz_84(14) <= zz_83;
    zz_84(13) <= zz_83;
    zz_84(12) <= zz_83;
    zz_84(11) <= zz_83;
    zz_84(10) <= zz_83;
    zz_84(9) <= zz_83;
    zz_84(8) <= zz_83;
    zz_84(7) <= zz_83;
    zz_84(6) <= zz_83;
    zz_84(5) <= zz_83;
    zz_84(4) <= zz_83;
    zz_84(3) <= zz_83;
    zz_84(2) <= zz_83;
    zz_84(1) <= zz_83;
    zz_84(0) <= zz_83;
  end process;

  zz_85 <= pkg_extract(pkg_cat(pkg_extract(decode_INSTRUCTION,31,25),pkg_extract(decode_INSTRUCTION,11,7)),11);
  process(zz_85)
  begin
    zz_86(19) <= zz_85;
    zz_86(18) <= zz_85;
    zz_86(17) <= zz_85;
    zz_86(16) <= zz_85;
    zz_86(15) <= zz_85;
    zz_86(14) <= zz_85;
    zz_86(13) <= zz_85;
    zz_86(12) <= zz_85;
    zz_86(11) <= zz_85;
    zz_86(10) <= zz_85;
    zz_86(9) <= zz_85;
    zz_86(8) <= zz_85;
    zz_86(7) <= zz_85;
    zz_86(6) <= zz_85;
    zz_86(5) <= zz_85;
    zz_86(4) <= zz_85;
    zz_86(3) <= zz_85;
    zz_86(2) <= zz_85;
    zz_86(1) <= zz_85;
    zz_86(0) <= zz_85;
  end process;

  process(decode_SRC2_CTRL,zz_23,zz_84,decode_INSTRUCTION,zz_86,zz_22)
  begin
    case decode_SRC2_CTRL is
      when Src2CtrlEnum_defaultEncoding_RS =>
        zz_87 <= zz_23;
      when Src2CtrlEnum_defaultEncoding_IMI =>
        zz_87 <= pkg_cat(zz_84,pkg_extract(decode_INSTRUCTION,31,20));
      when Src2CtrlEnum_defaultEncoding_IMS =>
        zz_87 <= pkg_cat(zz_86,pkg_cat(pkg_extract(decode_INSTRUCTION,31,25),pkg_extract(decode_INSTRUCTION,11,7)));
      when others =>
        zz_87 <= std_logic_vector(zz_22);
    end case;
  end process;

  process(execute_SRC1,execute_SRC_USE_SUB_LESS,execute_SRC2,execute_SRC2_FORCE_ZERO)
  begin
    execute_SrcPlugin_addSub <= std_logic_vector(((signed(execute_SRC1) + signed(pkg_mux(execute_SRC_USE_SUB_LESS,pkg_not(execute_SRC2),execute_SRC2))) + pkg_mux(execute_SRC_USE_SUB_LESS,pkg_signed("00000000000000000000000000000001"),pkg_signed("00000000000000000000000000000000"))));
    if execute_SRC2_FORCE_ZERO = '1' then
      execute_SrcPlugin_addSub <= execute_SRC1;
    end if;
  end process;

  execute_SrcPlugin_less <= pkg_mux(pkg_toStdLogic(pkg_extract(execute_SRC1,31) = pkg_extract(execute_SRC2,31)),pkg_extract(execute_SrcPlugin_addSub,31),pkg_mux(execute_SRC_LESS_UNSIGNED,pkg_extract(execute_SRC2,31),pkg_extract(execute_SRC1,31)));
  execute_LightShifterPlugin_isShift <= pkg_toStdLogic(execute_SHIFT_CTRL /= ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  execute_LightShifterPlugin_amplitude <= pkg_mux(execute_LightShifterPlugin_isActive,execute_LightShifterPlugin_amplitudeReg,unsigned(pkg_extract(execute_SRC2,4,0)));
  execute_LightShifterPlugin_shiftInput <= pkg_mux(execute_LightShifterPlugin_isActive,memory_REGFILE_WRITE_DATA,execute_SRC1);
  execute_LightShifterPlugin_done <= pkg_toStdLogic(pkg_extract(execute_LightShifterPlugin_amplitude,4,1) = pkg_unsigned("0000"));
  process(execute_SHIFT_CTRL,execute_LightShifterPlugin_shiftInput)
  begin
    case execute_SHIFT_CTRL is
      when ShiftCtrlEnum_defaultEncoding_SLL_1 =>
        zz_88 <= std_logic_vector(shift_left(unsigned(execute_LightShifterPlugin_shiftInput),1));
      when others =>
        zz_88 <= std_logic_vector(pkg_shiftRight(signed(pkg_cat(pkg_toStdLogicVector((pkg_toStdLogic(execute_SHIFT_CTRL = ShiftCtrlEnum_defaultEncoding_SRA_1) and pkg_extract(execute_LightShifterPlugin_shiftInput,31))),execute_LightShifterPlugin_shiftInput)),1));
    end case;
  end process;

  process(zz_91,zz_92,decode_INSTRUCTION,zz_113,zz_114,writeBack_INSTRUCTION,zz_115,zz_116,memory_INSTRUCTION,zz_117,zz_118,execute_INSTRUCTION,decode_RS1_USE)
  begin
    zz_89 <= pkg_toStdLogic(false);
    if zz_91 = '1' then
      if pkg_toStdLogic(zz_92 = pkg_extract(decode_INSTRUCTION,19,15)) = '1' then
        zz_89 <= pkg_toStdLogic(true);
      end if;
    end if;
    if zz_113 = '1' then
      if zz_114 = '1' then
        if pkg_toStdLogic(pkg_extract(writeBack_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15)) = '1' then
          zz_89 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if zz_115 = '1' then
      if zz_116 = '1' then
        if pkg_toStdLogic(pkg_extract(memory_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15)) = '1' then
          zz_89 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if zz_117 = '1' then
      if zz_118 = '1' then
        if pkg_toStdLogic(pkg_extract(execute_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,19,15)) = '1' then
          zz_89 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if (not decode_RS1_USE) = '1' then
      zz_89 <= pkg_toStdLogic(false);
    end if;
  end process;

  process(zz_91,zz_92,decode_INSTRUCTION,zz_113,zz_114,writeBack_INSTRUCTION,zz_115,zz_116,memory_INSTRUCTION,zz_117,zz_118,execute_INSTRUCTION,decode_RS2_USE)
  begin
    zz_90 <= pkg_toStdLogic(false);
    if zz_91 = '1' then
      if pkg_toStdLogic(zz_92 = pkg_extract(decode_INSTRUCTION,24,20)) = '1' then
        zz_90 <= pkg_toStdLogic(true);
      end if;
    end if;
    if zz_113 = '1' then
      if zz_114 = '1' then
        if pkg_toStdLogic(pkg_extract(writeBack_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20)) = '1' then
          zz_90 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if zz_115 = '1' then
      if zz_116 = '1' then
        if pkg_toStdLogic(pkg_extract(memory_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20)) = '1' then
          zz_90 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if zz_117 = '1' then
      if zz_118 = '1' then
        if pkg_toStdLogic(pkg_extract(execute_INSTRUCTION,11,7) = pkg_extract(decode_INSTRUCTION,24,20)) = '1' then
          zz_90 <= pkg_toStdLogic(true);
        end if;
      end if;
    end if;
    if (not decode_RS2_USE) = '1' then
      zz_90 <= pkg_toStdLogic(false);
    end if;
  end process;

  execute_BranchPlugin_eq <= pkg_toStdLogic(execute_SRC1 = execute_SRC2);
  zz_93 <= pkg_extract(execute_INSTRUCTION,14,12);
  process(zz_93,execute_BranchPlugin_eq,execute_SRC_LESS)
  begin
    if (zz_93 = pkg_stdLogicVector("000")) then
        zz_94 <= execute_BranchPlugin_eq;
    elsif (zz_93 = pkg_stdLogicVector("001")) then
        zz_94 <= (not execute_BranchPlugin_eq);
    elsif (pkg_toStdLogic((zz_93 and pkg_stdLogicVector("101")) = pkg_stdLogicVector("101")) = '1') then
        zz_94 <= (not execute_SRC_LESS);
    else
        zz_94 <= execute_SRC_LESS;
    end if;
  end process;

  process(execute_BRANCH_CTRL,zz_94)
  begin
    case execute_BRANCH_CTRL is
      when BranchCtrlEnum_defaultEncoding_INC =>
        zz_95 <= pkg_toStdLogic(false);
      when BranchCtrlEnum_defaultEncoding_JAL =>
        zz_95 <= pkg_toStdLogic(true);
      when BranchCtrlEnum_defaultEncoding_JALR =>
        zz_95 <= pkg_toStdLogic(true);
      when others =>
        zz_95 <= zz_94;
    end case;
  end process;

  execute_BranchPlugin_branch_src1 <= pkg_mux(pkg_toStdLogic(execute_BRANCH_CTRL = BranchCtrlEnum_defaultEncoding_JALR),unsigned(execute_RS1),execute_PC);
  zz_96 <= pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_extract(execute_INSTRUCTION,19,12)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,20))),pkg_extract(execute_INSTRUCTION,30,21)),19);
  process(zz_96)
  begin
    zz_97(10) <= zz_96;
    zz_97(9) <= zz_96;
    zz_97(8) <= zz_96;
    zz_97(7) <= zz_96;
    zz_97(6) <= zz_96;
    zz_97(5) <= zz_96;
    zz_97(4) <= zz_96;
    zz_97(3) <= zz_96;
    zz_97(2) <= zz_96;
    zz_97(1) <= zz_96;
    zz_97(0) <= zz_96;
  end process;

  zz_98 <= pkg_extract(pkg_extract(execute_INSTRUCTION,31,20),11);
  process(zz_98)
  begin
    zz_99(19) <= zz_98;
    zz_99(18) <= zz_98;
    zz_99(17) <= zz_98;
    zz_99(16) <= zz_98;
    zz_99(15) <= zz_98;
    zz_99(14) <= zz_98;
    zz_99(13) <= zz_98;
    zz_99(12) <= zz_98;
    zz_99(11) <= zz_98;
    zz_99(10) <= zz_98;
    zz_99(9) <= zz_98;
    zz_99(8) <= zz_98;
    zz_99(7) <= zz_98;
    zz_99(6) <= zz_98;
    zz_99(5) <= zz_98;
    zz_99(4) <= zz_98;
    zz_99(3) <= zz_98;
    zz_99(2) <= zz_98;
    zz_99(1) <= zz_98;
    zz_99(0) <= zz_98;
  end process;

  zz_100 <= pkg_extract(pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,7))),pkg_extract(execute_INSTRUCTION,30,25)),pkg_extract(execute_INSTRUCTION,11,8)),11);
  process(zz_100)
  begin
    zz_101(18) <= zz_100;
    zz_101(17) <= zz_100;
    zz_101(16) <= zz_100;
    zz_101(15) <= zz_100;
    zz_101(14) <= zz_100;
    zz_101(13) <= zz_100;
    zz_101(12) <= zz_100;
    zz_101(11) <= zz_100;
    zz_101(10) <= zz_100;
    zz_101(9) <= zz_100;
    zz_101(8) <= zz_100;
    zz_101(7) <= zz_100;
    zz_101(6) <= zz_100;
    zz_101(5) <= zz_100;
    zz_101(4) <= zz_100;
    zz_101(3) <= zz_100;
    zz_101(2) <= zz_100;
    zz_101(1) <= zz_100;
    zz_101(0) <= zz_100;
  end process;

  process(execute_BRANCH_CTRL,zz_97,execute_INSTRUCTION,zz_99,zz_101)
  begin
    case execute_BRANCH_CTRL is
      when BranchCtrlEnum_defaultEncoding_JAL =>
        zz_102 <= pkg_cat(pkg_cat(zz_97,pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_extract(execute_INSTRUCTION,19,12)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,20))),pkg_extract(execute_INSTRUCTION,30,21))),pkg_toStdLogicVector(pkg_toStdLogic(false)));
      when BranchCtrlEnum_defaultEncoding_JALR =>
        zz_102 <= pkg_cat(zz_99,pkg_extract(execute_INSTRUCTION,31,20));
      when others =>
        zz_102 <= pkg_cat(pkg_cat(zz_101,pkg_cat(pkg_cat(pkg_cat(pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,31)),pkg_toStdLogicVector(pkg_extract(execute_INSTRUCTION,7))),pkg_extract(execute_INSTRUCTION,30,25)),pkg_extract(execute_INSTRUCTION,11,8))),pkg_toStdLogicVector(pkg_toStdLogic(false)));
    end case;
  end process;

  execute_BranchPlugin_branch_src2 <= unsigned(zz_102);
  execute_BranchPlugin_branchAdder <= (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  BranchPlugin_jumpInterface_valid <= ((memory_arbitration_isValid and memory_BRANCH_DO) and (not pkg_toStdLogic(false)));
  BranchPlugin_jumpInterface_payload <= memory_BRANCH_CALC;
  zz_26 <= zz_38;
  zz_24 <= zz_35;
  zz_19 <= decode_BRANCH_CTRL;
  zz_17 <= zz_32;
  zz_20 <= decode_to_execute_BRANCH_CTRL;
  zz_16 <= decode_ENV_CTRL;
  zz_13 <= execute_ENV_CTRL;
  zz_11 <= memory_ENV_CTRL;
  zz_14 <= zz_33;
  zz_41 <= decode_to_execute_ENV_CTRL;
  zz_40 <= execute_to_memory_ENV_CTRL;
  zz_42 <= memory_to_writeBack_ENV_CTRL;
  zz_9 <= decode_SHIFT_CTRL;
  zz_7 <= zz_34;
  zz_21 <= decode_to_execute_SHIFT_CTRL;
  zz_6 <= decode_ALU_BITWISE_CTRL;
  zz_4 <= zz_37;
  zz_28 <= decode_to_execute_ALU_BITWISE_CTRL;
  zz_3 <= decode_ALU_CTRL;
  zz_1 <= zz_36;
  zz_27 <= decode_to_execute_ALU_CTRL;
  decode_arbitration_isFlushed <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushNext),pkg_toStdLogicVector(execute_arbitration_flushNext))) /= pkg_stdLogicVector("000")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(execute_arbitration_flushIt),pkg_toStdLogicVector(decode_arbitration_flushIt)))) /= pkg_stdLogicVector("0000")));
  execute_arbitration_isFlushed <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushNext),pkg_toStdLogicVector(memory_arbitration_flushNext)) /= pkg_stdLogicVector("00")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_cat(pkg_toStdLogicVector(memory_arbitration_flushIt),pkg_toStdLogicVector(execute_arbitration_flushIt))) /= pkg_stdLogicVector("000")));
  memory_arbitration_isFlushed <= (pkg_toStdLogic(pkg_toStdLogicVector(writeBack_arbitration_flushNext) /= pkg_stdLogicVector("0")) or pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(writeBack_arbitration_flushIt),pkg_toStdLogicVector(memory_arbitration_flushIt)) /= pkg_stdLogicVector("00")));
  writeBack_arbitration_isFlushed <= (pkg_toStdLogic(false) or pkg_toStdLogic(pkg_toStdLogicVector(writeBack_arbitration_flushIt) /= pkg_stdLogicVector("0")));
  decode_arbitration_isStuckByOthers <= (decode_arbitration_haltByOther or (((pkg_toStdLogic(false) or execute_arbitration_isStuck) or memory_arbitration_isStuck) or writeBack_arbitration_isStuck));
  decode_arbitration_isStuck <= (decode_arbitration_haltItself or decode_arbitration_isStuckByOthers);
  decode_arbitration_isMoving <= ((not decode_arbitration_isStuck) and (not decode_arbitration_removeIt));
  decode_arbitration_isFiring <= ((decode_arbitration_isValid and (not decode_arbitration_isStuck)) and (not decode_arbitration_removeIt));
  execute_arbitration_isStuckByOthers <= (execute_arbitration_haltByOther or ((pkg_toStdLogic(false) or memory_arbitration_isStuck) or writeBack_arbitration_isStuck));
  execute_arbitration_isStuck <= (execute_arbitration_haltItself or execute_arbitration_isStuckByOthers);
  execute_arbitration_isMoving <= ((not execute_arbitration_isStuck) and (not execute_arbitration_removeIt));
  execute_arbitration_isFiring <= ((execute_arbitration_isValid and (not execute_arbitration_isStuck)) and (not execute_arbitration_removeIt));
  memory_arbitration_isStuckByOthers <= (memory_arbitration_haltByOther or (pkg_toStdLogic(false) or writeBack_arbitration_isStuck));
  memory_arbitration_isStuck <= (memory_arbitration_haltItself or memory_arbitration_isStuckByOthers);
  memory_arbitration_isMoving <= ((not memory_arbitration_isStuck) and (not memory_arbitration_removeIt));
  memory_arbitration_isFiring <= ((memory_arbitration_isValid and (not memory_arbitration_isStuck)) and (not memory_arbitration_removeIt));
  writeBack_arbitration_isStuckByOthers <= (writeBack_arbitration_haltByOther or pkg_toStdLogic(false));
  writeBack_arbitration_isStuck <= (writeBack_arbitration_haltItself or writeBack_arbitration_isStuckByOthers);
  writeBack_arbitration_isMoving <= ((not writeBack_arbitration_isStuck) and (not writeBack_arbitration_removeIt));
  writeBack_arbitration_isFiring <= ((writeBack_arbitration_isValid and (not writeBack_arbitration_isStuck)) and (not writeBack_arbitration_removeIt));
  process(clk, reset)
  begin
    if reset = '1' then
      IBusSimplePlugin_fetchPc_pcReg <= pkg_unsigned("10000000000000000000000000000000");
      IBusSimplePlugin_fetchPc_booted <= pkg_toStdLogic(false);
      IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      zz_50 <= pkg_toStdLogic(false);
      zz_51 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_nextPcCalc_valids_4 <= pkg_toStdLogic(false);
      IBusSimplePlugin_injector_decodeRemoved <= pkg_toStdLogic(false);
      IBusSimplePlugin_pendingCmd <= pkg_unsigned("000");
      IBusSimplePlugin_rspJoin_discardCounter <= pkg_unsigned("000");
      CsrPlugin_mstatus_MIE <= pkg_toStdLogic(false);
      CsrPlugin_mstatus_MPIE <= pkg_toStdLogic(false);
      CsrPlugin_mstatus_MPP <= pkg_unsigned("11");
      CsrPlugin_mie_MEIE <= pkg_toStdLogic(false);
      CsrPlugin_mie_MTIE <= pkg_toStdLogic(false);
      CsrPlugin_mie_MSIE <= pkg_toStdLogic(false);
      CsrPlugin_interrupt_valid <= pkg_toStdLogic(false);
      CsrPlugin_hadException <= pkg_toStdLogic(false);
      execute_CsrPlugin_wfiWake <= pkg_toStdLogic(false);
      zz_80 <= pkg_toStdLogic(true);
      execute_LightShifterPlugin_isActive <= pkg_toStdLogic(false);
      zz_91 <= pkg_toStdLogic(false);
      execute_arbitration_isValid <= pkg_toStdLogic(false);
      memory_arbitration_isValid <= pkg_toStdLogic(false);
      writeBack_arbitration_isValid <= pkg_toStdLogic(false);
      memory_to_writeBack_REGFILE_WRITE_DATA <= pkg_stdLogicVector("00000000000000000000000000000000");
      memory_to_writeBack_INSTRUCTION <= pkg_stdLogicVector("00000000000000000000000000000000");
    elsif rising_edge(clk) then
      IBusSimplePlugin_fetchPc_booted <= pkg_toStdLogic(true);
      if (IBusSimplePlugin_fetchPc_corrected or IBusSimplePlugin_fetchPc_pcRegPropagate) = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      end if;
      if (IBusSimplePlugin_fetchPc_output_valid and IBusSimplePlugin_fetchPc_output_ready) = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(true);
      end if;
      if ((not IBusSimplePlugin_fetchPc_output_valid) and IBusSimplePlugin_fetchPc_output_ready) = '1' then
        IBusSimplePlugin_fetchPc_inc <= pkg_toStdLogic(false);
      end if;
      if (IBusSimplePlugin_fetchPc_booted and ((IBusSimplePlugin_fetchPc_output_ready or IBusSimplePlugin_fetcherflushIt) or IBusSimplePlugin_fetchPc_pcRegPropagate)) = '1' then
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        zz_50 <= pkg_toStdLogic(false);
      end if;
      if zz_48 = '1' then
        zz_50 <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end if;
      if IBusSimplePlugin_iBusRsp_output_ready = '1' then
        zz_51 <= IBusSimplePlugin_iBusRsp_output_valid;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        zz_51 <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(false);
      end if;
      if (not (not IBusSimplePlugin_iBusRsp_stages_1_input_ready)) = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= pkg_toStdLogic(true);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= pkg_toStdLogic(false);
      end if;
      if (not (not IBusSimplePlugin_injector_decodeInput_ready)) = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= pkg_toStdLogic(false);
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= pkg_toStdLogic(false);
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= pkg_toStdLogic(false);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= pkg_toStdLogic(false);
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= pkg_toStdLogic(false);
      end if;
      if decode_arbitration_removeIt = '1' then
        IBusSimplePlugin_injector_decodeRemoved <= pkg_toStdLogic(true);
      end if;
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_injector_decodeRemoved <= pkg_toStdLogic(false);
      end if;
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - pkg_resize(unsigned(pkg_toStdLogicVector((iBus_rsp_valid and pkg_toStdLogic(IBusSimplePlugin_rspJoin_discardCounter /= pkg_unsigned("000"))))),3));
      if IBusSimplePlugin_fetcherflushIt = '1' then
        IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_pendingCmd - pkg_resize(unsigned(pkg_toStdLogicVector(iBus_rsp_valid)),3));
      end if;
      CsrPlugin_interrupt_valid <= pkg_toStdLogic(false);
      if zz_119 = '1' then
        if zz_120 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if zz_121 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
        if zz_122 = '1' then
          CsrPlugin_interrupt_valid <= pkg_toStdLogic(true);
        end if;
      end if;
      CsrPlugin_hadException <= CsrPlugin_exception;
      if zz_110 = '1' then
        case CsrPlugin_targetPrivilege is
          when "11" =>
            CsrPlugin_mstatus_MIE <= pkg_toStdLogic(false);
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          when others =>
        end case;
      end if;
      if zz_111 = '1' then
        case zz_112 is
          when "11" =>
            CsrPlugin_mstatus_MPP <= pkg_unsigned("00");
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= pkg_toStdLogic(true);
          when others =>
        end case;
      end if;
      execute_CsrPlugin_wfiWake <= (pkg_toStdLogic(pkg_cat(pkg_toStdLogicVector(zz_66),pkg_cat(pkg_toStdLogicVector(zz_65),pkg_toStdLogicVector(zz_64))) /= pkg_stdLogicVector("000")) or CsrPlugin_thirdPartyWake);
      zz_80 <= pkg_toStdLogic(false);
      if zz_108 = '1' then
        if zz_109 = '1' then
          execute_LightShifterPlugin_isActive <= pkg_toStdLogic(true);
          if execute_LightShifterPlugin_done = '1' then
            execute_LightShifterPlugin_isActive <= pkg_toStdLogic(false);
          end if;
        end if;
      end if;
      if execute_arbitration_removeIt = '1' then
        execute_LightShifterPlugin_isActive <= pkg_toStdLogic(false);
      end if;
      zz_91 <= (zz_30 and writeBack_arbitration_isFiring);
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_REGFILE_WRITE_DATA <= memory_REGFILE_WRITE_DATA;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end if;
      if ((not execute_arbitration_isStuck) or execute_arbitration_removeIt) = '1' then
        execute_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if ((not decode_arbitration_isStuck) and (not decode_arbitration_removeIt)) = '1' then
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end if;
      if ((not memory_arbitration_isStuck) or memory_arbitration_removeIt) = '1' then
        memory_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if ((not execute_arbitration_isStuck) and (not execute_arbitration_removeIt)) = '1' then
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end if;
      if ((not writeBack_arbitration_isStuck) or writeBack_arbitration_removeIt) = '1' then
        writeBack_arbitration_isValid <= pkg_toStdLogic(false);
      end if;
      if ((not memory_arbitration_isStuck) and (not memory_arbitration_removeIt)) = '1' then
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end if;
      case execute_CsrPlugin_csrAddress is
        when "001100000000" =>
          if execute_CsrPlugin_writeEnable = '1' then
            CsrPlugin_mstatus_MPP <= unsigned(pkg_extract(execute_CsrPlugin_writeData,12,11));
            CsrPlugin_mstatus_MPIE <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,7,7),0);
            CsrPlugin_mstatus_MIE <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,3,3),0);
          end if;
        when "001101000100" =>
        when "001100000100" =>
          if execute_CsrPlugin_writeEnable = '1' then
            CsrPlugin_mie_MEIE <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,11,11),0);
            CsrPlugin_mie_MTIE <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,7,7),0);
            CsrPlugin_mie_MSIE <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,3,3),0);
          end if;
        when "001101000010" =>
        when others =>
      end case;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if IBusSimplePlugin_iBusRsp_output_ready = '1' then
        zz_52 <= IBusSimplePlugin_iBusRsp_output_payload_pc;
        zz_53 <= IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
        zz_54 <= IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
        zz_55 <= IBusSimplePlugin_iBusRsp_output_payload_isRvc;
      end if;
      if IBusSimplePlugin_injector_decodeInput_ready = '1' then
        IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
      end if;
      assert (not (((dBus_rsp_ready and memory_MEMORY_ENABLE) and memory_arbitration_isValid) and memory_arbitration_isStuck)) = '1' report "DBusSimplePlugin doesn't allow memory stage stall when read happend"  severity ERROR;
      assert (not (((writeBack_arbitration_isValid and writeBack_MEMORY_ENABLE) and (not writeBack_MEMORY_STORE)) and writeBack_arbitration_isStuck)) = '1' report "DBusSimplePlugin doesn't allow writeback stage stall when read happend"  severity ERROR;
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      CsrPlugin_mip_MSIP <= softwareInterrupt;
      CsrPlugin_mcycle <= (CsrPlugin_mcycle + pkg_unsigned("0000000000000000000000000000000000000000000000000000000000000001"));
      if writeBack_arbitration_isFiring = '1' then
        CsrPlugin_minstret <= (CsrPlugin_minstret + pkg_unsigned("0000000000000000000000000000000000000000000000000000000000000001"));
      end if;
      if zz_119 = '1' then
        if zz_120 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("0111");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if zz_121 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("0011");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
        if zz_122 = '1' then
          CsrPlugin_interrupt_code <= pkg_unsigned("1011");
          CsrPlugin_interrupt_targetPrivilege <= pkg_unsigned("11");
        end if;
      end if;
      if zz_110 = '1' then
        case CsrPlugin_targetPrivilege is
          when "11" =>
            CsrPlugin_mcause_interrupt <= (not CsrPlugin_hadException);
            CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
            CsrPlugin_mepc <= decode_PC;
          when others =>
        end case;
      end if;
      if zz_108 = '1' then
        if zz_109 = '1' then
          execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - pkg_unsigned("00001"));
        end if;
      end if;
      zz_92 <= pkg_extract(zz_29,11,7);
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_PC <= zz_22;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_PC <= execute_PC;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_PC <= memory_PC;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_RS2 <= zz_23;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_IS_CSR <= decode_IS_CSR;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_BRANCH_CTRL <= zz_18;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_ENV_CTRL <= zz_15;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_ENV_CTRL <= zz_12;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_ENV_CTRL <= zz_10;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
      end if;
      if ((not memory_arbitration_isStuck) and (not execute_arbitration_isStuckByOthers)) = '1' then
        execute_to_memory_REGFILE_WRITE_DATA <= zz_39;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SRC1 <= decode_SRC1;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SHIFT_CTRL <= zz_8;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_ALU_BITWISE_CTRL <= zz_5;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_RS1 <= zz_25;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_ALU_CTRL <= zz_2;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
      end if;
      if (not memory_arbitration_isStuck) = '1' then
        execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_FORMAL_PC_NEXT <= zz_44;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
      end if;
      if (not writeBack_arbitration_isStuck) = '1' then
        memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
      end if;
      if (not execute_arbitration_isStuck) = '1' then
        decode_to_execute_SRC2 <= decode_SRC2;
      end if;
      case execute_CsrPlugin_csrAddress is
        when "001100000000" =>
        when "001101000100" =>
          if execute_CsrPlugin_writeEnable = '1' then
            CsrPlugin_mip_MSIP <= pkg_extract(pkg_extract(execute_CsrPlugin_writeData,3,3),0);
          end if;
        when "001100000100" =>
        when "001101000010" =>
        when others =>
      end case;
    end if;
  end process;

end arch;

