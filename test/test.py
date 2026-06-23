# cocotb 최소 sanity 테스트 (GDS CI 용). 풀 기능검증은 tb_reference_full.v(Verilog) 참고.
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_reset_and_run(dut):
    """리셋 후 크래시 없이 동작 + uo_out 읽힘 확인 (최소 sanity)."""
    dut._log.info("start — 20 MHz clock")
    cocotb.start_soon(Clock(dut.clk, 50, units="ns").start())  # 50 ns = 20 MHz

    dut.ena.value = 1
    dut.ui_in.value = 0x80      # ui[7]=arm_enable
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 500)

    # 동작 지속(크래시/락업 없음) + 출력 읽기
    val = int(dut.uo_out.value)
    dut._log.info(f"uo_out after reset = {val:#04x}")
    await ClockCycles(dut.clk, 20000)
    dut._log.info(f"uo_out later        = {int(dut.uo_out.value):#04x}")
    dut._log.info("ran without crash ✔  (full functional test: tb_reference_full.v)")
