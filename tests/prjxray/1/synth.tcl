# Support both the upstream spelling and the existing local filename rename.
set cpu quasiSoC/rtl/pcpu/riscv-multicyc.v
if {![file exists $cpu]} {
    set cpu quasiSoC/rtl/pcpu/riscv_multicyc.v
}
yosys read_verilog -I. $cpu quasiSoC/rtl/pcpu/privilege.v quasiSoC/rtl/pcpu/rv32m.v quasiSoC/rtl/pcpu/alu.v quasiSoC/rtl/pcpu/register_file.v
yosys hierarchy -check -top riscv_multicyc
