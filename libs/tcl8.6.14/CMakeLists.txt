cmake_minimum_required(VERSION 3.26...3.30)

project(
  tcl
  VERSION 1.0
  LANGUAGES C)

set (GENERIC_OBJS
    generic/regcomp.c
    generic/regexec.c
    generic/regfree.c
    generic/regerror.c
    generic/tclAlloc.c
    generic/tclAssembly.c
    generic/tclAsync.c
    generic/tclBasic.c
    generic/tclBinary.c
    generic/tclCkalloc.c
    generic/tclClock.c
    generic/tclCmdAH.c
    generic/tclCmdIL.c
    generic/tclCmdMZ.c
    generic/tclCompCmds.c
    generic/tclCompCmdsGR.c
    generic/tclCompCmdsSZ.c
    generic/tclCompExpr.c
    generic/tclCompile.c
    generic/tclConfig.c
    generic/tclDate.c
    generic/tclDictObj.c
    generic/tclDisassemble.c
    generic/tclEncoding.c
    generic/tclEnsemble.c
    generic/tclEnv.c
    generic/tclEvent.c
    generic/tclExecute.c
    generic/tclFCmd.c
    generic/tclFileName.c
    generic/tclGet.c
    generic/tclHash.c
    generic/tclHistory.c
    generic/tclIndexObj.c
    generic/tclInterp.c
    generic/tclIO.c
    generic/tclIOCmd.c
    generic/tclIOGT.c
    generic/tclIORChan.c
    generic/tclIORTrans.c
    generic/tclIOSock.c
    generic/tclIOUtil.c
    generic/tclLink.c
    generic/tclLiteral.c
    generic/tclListObj.c
    generic/tclLoad.c
#    generic/tclMain.c
    generic/tclNamesp.c
    generic/tclNotify.c
    generic/tclOO.c
    generic/tclOOBasic.c
    generic/tclOOCall.c
    generic/tclOODefineCmds.c
    generic/tclOOInfo.c
    generic/tclOOMethod.c
    generic/tclOOStubInit.c
    generic/tclObj.c
    generic/tclOptimize.c
    generic/tclPanic.c
    generic/tclParse.c
    generic/tclPathObj.c
    generic/tclPipe.c
    generic/tclPkg.c
    generic/tclPkgConfig.c
    generic/tclPosixStr.c
    generic/tclPreserve.c
    generic/tclProc.c
    generic/tclRegexp.c
    generic/tclResolve.c
    generic/tclResult.c
    generic/tclScan.c
    generic/tclStringObj.c
    generic/tclStrToD.c
    generic/tclStubInit.c
    generic/tclThread.c
    generic/tclThreadAlloc.c
    generic/tclThreadJoin.c
    generic/tclThreadStorage.c
    generic/tclTimer.c
    generic/tclTomMathInterface.c
    generic/tclTrace.c
    generic/tclUtf.c
    generic/tclUtil.c
    generic/tclVar.c
    generic/tclZlib.c
)

set (TOMMATH_OBJS
    libtommath/bn_mp_add.c
    libtommath/bn_mp_add_d.c
    libtommath/bn_mp_and.c
    libtommath/bn_mp_clamp.c
    libtommath/bn_mp_clear.c
    libtommath/bn_mp_clear_multi.c
    libtommath/bn_mp_cmp.c
    libtommath/bn_mp_cmp_d.c
    libtommath/bn_mp_cmp_mag.c
    libtommath/bn_mp_cnt_lsb.c
    libtommath/bn_mp_copy.c
    libtommath/bn_mp_count_bits.c
    libtommath/bn_mp_div.c
    libtommath/bn_mp_div_d.c
    libtommath/bn_mp_div_2.c
    libtommath/bn_mp_div_2d.c
    libtommath/bn_mp_div_3.c
    libtommath/bn_mp_exch.c
    libtommath/bn_mp_expt_u32.c
    libtommath/bn_mp_grow.c
    libtommath/bn_mp_init.c
    libtommath/bn_mp_init_copy.c
    libtommath/bn_mp_init_multi.c
    libtommath/bn_mp_init_set.c
    libtommath/bn_mp_init_size.c
    libtommath/bn_mp_lshd.c
    libtommath/bn_mp_mod.c
    libtommath/bn_mp_mod_2d.c
    libtommath/bn_mp_mul.c
    libtommath/bn_mp_mul_2.c
    libtommath/bn_mp_mul_2d.c
    libtommath/bn_mp_mul_d.c
    libtommath/bn_mp_neg.c
    libtommath/bn_mp_or.c
    libtommath/bn_mp_pack.c
    libtommath/bn_mp_pack_count.c
    libtommath/bn_mp_radix_size.c
    libtommath/bn_mp_radix_smap.c
    libtommath/bn_mp_read_radix.c
    libtommath/bn_mp_rshd.c
    libtommath/bn_mp_set.c
    libtommath/bn_mp_shrink.c
    libtommath/bn_mp_sqr.c
    libtommath/bn_mp_sqrt.c
    libtommath/bn_mp_sub.c
    libtommath/bn_mp_sub_d.c
    libtommath/bn_mp_signed_rsh.c
    libtommath/bn_mp_to_ubin.c
    libtommath/bn_mp_to_radix.c
    libtommath/bn_mp_ubin_size.c
    libtommath/bn_mp_unpack.c
    libtommath/bn_mp_xor.c
    libtommath/bn_mp_zero.c
    libtommath/bn_s_mp_add.c
    libtommath/bn_s_mp_balance_mul.c
    libtommath/bn_s_mp_karatsuba_mul.c
    libtommath/bn_s_mp_karatsuba_sqr.c
    libtommath/bn_s_mp_mul_digs.c
    libtommath/bn_s_mp_mul_digs_fast.c
    libtommath/bn_s_mp_reverse.c
    libtommath/bn_s_mp_sqr_fast.c
    libtommath/bn_s_mp_sqr.c
    libtommath/bn_s_mp_sub.c
    libtommath/bn_s_mp_toom_mul.c
    libtommath/bn_s_mp_toom_sqr.c
)

set (WIN_OBJS
    win/tclWin32Dll.c
    win/tclWinChan.c
    win/tclWinConsole.c
    win/tclWinSerial.c
    win/tclWinError.c
    win/tclWinFCmd.c
    win/tclWinFile.c
    win/tclWinInit.c
    win/tclWinLoad.c
    win/tclWinNotify.c
    win/tclWinPipe.c
    win/tclWinSock.c
    win/tclWinThrd.c
    win/tclWinTime.c
    win/tclMain.c
    win/tclWinDde.c
    win/tclWinReg.c
)

set (UNIX_OBJS
    unix/tclUnixChan.c
    unix/tclUnixCompat.c
    unix/tclUnixEvent.c
    unix/tclUnixTest.c
    unix/tclUnixFCmd.c
    unix/tclUnixFile.c
    unix/tclUnixInit.c
    unix/tclUnixNotfy.c
    unix/tclUnixPipe.c
    unix/tclUnixSock.c
    unix/tclUnixThrd.c
    unix/tclUnixTime.c
    unix/tclLoadDl.c
    unix/tclLoadDyld.c
    generic/tclMain.c
)

#    set (STUB_OBJS
#    win/tclStubLib.c
#    win/tclTomMathStubLib.c
#    win/tclOOStubLib.c
#    )

#    set (TCLSH_OBJS
#    tclAppInit.c
#    )

set (ZLIB_OBJS
    compat/zlib/adler32.c
    compat/zlib/compress.c
    compat/zlib/crc32.c
    compat/zlib/deflate.c
    compat/zlib/infback.c
    compat/zlib/inffast.c
    compat/zlib/inflate.c
    compat/zlib/inftrees.c
    compat/zlib/trees.c
    compat/zlib/uncompr.c
    compat/zlib/zutil.c
)

if(WIN32)
    add_library(tcl STATIC
        ${GENERIC_OBJS}
        ${TOMMATH_OBJS}
        ${WIN_OBJS}
        ${ZLIB_OBJS}
    )

    set_target_properties(tcl PROPERTIES LINKER_LANGUAGE C)

    target_include_directories(tcl
        PUBLIC
            generic
            win
            libtommath
    )

    add_definitions(-DHAVE_NO_SEH)
    add_definitions(-DSTATIC_BUILD)
    add_definitions(-DCFG_RUNTIME_LIBDIR="")
    add_definitions(-DCFG_RUNTIME_BINDIR="")
    add_definitions(-DCFG_RUNTIME_SCRDIR="")
    add_definitions(-DCFG_RUNTIME_INCDIR="")
    add_definitions(-DCFG_RUNTIME_DOCDIR="")
    add_definitions(-DCFG_INSTALL_LIBDIR="")
    add_definitions(-DCFG_INSTALL_BINDIR="")
    add_definitions(-DCFG_INSTALL_SCRDIR="")
    add_definitions(-DCFG_INSTALL_INCDIR="")
    add_definitions(-DCFG_INSTALL_DOCDIR="")
    add_definitions(-DBIN_INSTALL_DIR_NATIVE="")
    add_definitions(-DSCRIPT_INSTALL_DIR_NATIVE="")
    add_definitions(-DINCLUDE_INSTALL_DIR_NATIVE="")
    add_definitions(-DMAN_INSTALL_DIR_NATIVE="")
    add_definitions(-DBUILD_tcl="")
    add_definitions(-DMP_FIXED_CUTOFFS)
    add_definitions(-DMP_NO_STDINT)
else()
    add_library(tcl STATIC
        ${GENERIC_OBJS}
        ${TOMMATH_OBJS}
        ${UNIX_OBJS}
        ${ZLIB_OBJS}
    )

    set_target_properties(tcl PROPERTIES LINKER_LANGUAGE C)

    target_include_directories(tcl
        PUBLIC
            generic
            unix
            libtommath
            compat
    )

    add_definitions(-DHAVE_NO_SEH)
    add_definitions(-DSTATIC_BUILD)
    add_definitions(-DCFG_RUNTIME_LIBDIR="")
    add_definitions(-DCFG_RUNTIME_BINDIR="")
    add_definitions(-DCFG_RUNTIME_SCRDIR="")
    add_definitions(-DCFG_RUNTIME_INCDIR="")
    add_definitions(-DCFG_RUNTIME_DOCDIR="")
    add_definitions(-DCFG_INSTALL_LIBDIR="")
    add_definitions(-DCFG_INSTALL_BINDIR="")
    add_definitions(-DCFG_INSTALL_SCRDIR="")
    add_definitions(-DCFG_INSTALL_INCDIR="")
    add_definitions(-DCFG_INSTALL_DOCDIR="")
    add_definitions(-DBIN_INSTALL_DIR_NATIVE="")
    add_definitions(-DSCRIPT_INSTALL_DIR_NATIVE="")
    add_definitions(-DINCLUDE_INSTALL_DIR_NATIVE="")
    add_definitions(-DMAN_INSTALL_DIR_NATIVE="")
    add_definitions(-DBUILD_tcl="")
    add_definitions(-DMP_FIXED_CUTOFFS)
    add_definitions(-DMP_NO_STDINT)
    add_definitions(-DUSE_DIRENT2_H)
    add_definitions(-D_XOPEN_SOURCE_EXTENDED)
    add_definitions(-D_XOPEN_SOURCE)
    add_definitions(-D_ATFILE_SOURCE)
    add_definitions(-D_REENTRANT)
    add_definitions(-D_GNU_SOURCE)
    add_definitions(-D_BSD_SOURCE)
    add_definitions(-D_POSIX_SOURCE)
    add_definitions(-DTCL_LIBRARY="")
    add_definitions(-DTCL_PACKAGE_PATH="")
endif()
