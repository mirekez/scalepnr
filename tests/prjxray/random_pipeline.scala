//> using scala "2.13.12"
//> using dep "org.chipsalliance::chisel:6.5.0"
//> using plugin "org.chipsalliance:::chisel-plugin:6.5.0"

import _root_.circt.stage.ChiselStage
import chisel3._
import chisel3.reflect.DataMirror
import chisel3.util._
import java.io.{BufferedWriter, File, FileWriter}
import java.nio.file.{Files, Path, Paths}
import scala.collection.mutable.ListBuffer
import scala.io.Source
import scala.util.Random

import PnrTests.NodeFabric

/** The pnr_tests pipeline with an explicitly seeded NodeFabric generator. */
class SeededTestPipeline(
    cfg: Config,
    complexity: Int,
    length: Int,
    maxWidth: Int,
    maxChainLength: Int,
    seed: Long) extends TestPipelineIO(cfg.startWidth) {
  override def desiredName: String = "TestPipeline"

  val in = Wire(Flipped(Decoupled(UInt(cfg.startWidth.W))))
  in.valid := in_valid
  in_ready := in.ready
  in.bits := in_bits

  val out = Wire(Decoupled(UInt(cfg.startWidth.W)))
  out_valid := out.valid
  out.ready := out_ready
  out_bits := out.bits

  val fabric = new NodeFabric(cfg.startWidth)
  fabric.random.setSeed(seed)
  val modules = (0 until length).map(_ =>
    fabric.GenModule(maxWidth, cfg.minWidth, complexity, maxChainLength))
  fabric.ChainModules(modules, in, out, maxWidth, maxChainLength)
}

/** Generate the pnr_tests pin constraints from an independent seeded stream. */
class SeededXDCGen[T <: Module](
    moduleFactory: () => T,
    outdir: String,
    project: String,
    xrayPath: String,
    seed: Long) extends Module {
  val test = Module(moduleFactory())
  val pins = ListBuffer[String]()
  val clockPins = ListBuffer[String]()
  val sourceCSV = Source.fromFile(s"$xrayPath/package_pins.csv")
  sourceCSV.getLines().next()
  for (line <- sourceCSV.getLines()) {
    val columns = line.split(",")
    if (columns.length >= 5) {
      if (!columns(3).contains("GTP") && !columns(3).contains("MONITOR") &&
          !columns(4).contains("SRCC") && !columns(4).contains("MRCC")) {
        pins += columns(0)
      }
      if ((columns(4).contains("SRCC") || columns(4).contains("MRCC")) &&
          !columns(4).contains("N_T")) {
        clockPins += columns(0)
      }
    }
  }
  sourceCSV.close()

  val random = new Random(seed)
  val writer = new BufferedWriter(new FileWriter(new File(s"$outdir/$project.xdc")))
  def assignPin(name: String, clock: Boolean): Unit = {
    val available = if (clock) clockPins else pins
    require(available.nonEmpty, s"no package pin remains for $name")
    val index = random.nextInt(available.length)
    writer.write(s"set_property IOSTANDARD LVCMOS33 [get_ports $name]\n")
    writer.write(s"set_property PACKAGE_PIN ${available(index)} [get_ports $name]\n")
    available.remove(index)
  }
  DataMirror.modulePorts(test).foreach { case (name, port) =>
    port := DontCare
    if (port.getWidth > 1) {
      (0 until port.getWidth).foreach(bit => assignPin(s"$name[$bit]", false))
    } else {
      assignPin(name, name == "clock")
    }
  }
  writer.close()
}

/** Emit one pnr_tests pipeline without invoking its bundled nextpnr flow. */
object ScalepnrRandomPipeline extends App {
  require(args.length == 9,
    "usage: ScalepnrRandomPipeline OUT_DIR PROJECT PART DB_DIR COMPLEXITY SIZE MAX_WIDTH MAX_CHAIN_LENGTH SEED")

  val outDir: Path = Paths.get(args(0)).toAbsolutePath.normalize
  val project = args(1)
  val part = args(2)
  val dbDir = args(3)
  val complexity = args(4).toInt
  val size = args(5).toInt
  val maxWidth = args(6).toInt
  val maxChainLength = args(7).toInt
  val seed = args(8).toLong
  require(complexity >= 0, "complexity must be non-negative")
  require(size > 0, "size must be positive")
  require(maxWidth >= 32, "maximum width must be at least 32")
  require(maxChainLength > 0, "maximum chain length must be positive")
  Files.createDirectories(outDir)

  val cfg = new Config
  ChiselStage.emitSystemVerilog(
    new SeededXDCGen(
      () => new TestPipelineIO(cfg.startWidth), outDir.toString, project,
      dbDir, seed ^ 0x5deece66dL)
  )
  ChiselStage.emitSystemVerilogFile(
    new SeededTestPipeline(cfg, complexity, size, maxWidth, maxChainLength, seed),
    Array("--target-dir", outDir.toString),
    firtoolOpts = Array("--lowering-options=disallowLocalVariables,disallowPackedArrays")
  )
}
