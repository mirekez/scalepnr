#include "tcl_pnr.h"

int
TclPnr_Init(Tcl_Interp *interp)
{
    Tcl_ValueType t3ArgTypes[2];

    Tcl_Obj **objv, *objPtr;
    int objc, index;
    static const char *const specialOptions[] = {
"-appinitprocerror", "-appinitprocdeleteinterp",
"-appinitprocclosestderr", "-appinitprocsetrcfile", NULL
    };

    Tcl_CreateObjCommand(interp, "get_tiles", get_tiles_cmd, NULL, NULL);
    Tcl_CreateObjCommand(interp, "get_ports", get_ports_cmd, NULL, NULL);
    Tcl_CreateObjCommand(interp, "create_clock", create_clock_cmd, NULL, NULL);
    return 0;
}

int Tcl_AppInit(Tcl_Interp *interp)
{
    if ((Tcl_Init)(interp) == TCL_ERROR) {
        return TCL_ERROR;
    }

    if (TclPnr_Init(interp) == TCL_ERROR) {
        return TCL_ERROR;
    }

#ifdef DJGPP
    (Tcl_ObjSetVar2)(interp, Tcl_NewStringObj("tcl_rcFileName", -1), NULL,
    Tcl_NewStringObj("~/tclsh.rc", -1), TCL_GLOBAL_ONLY);
#else
    (Tcl_ObjSetVar2)(interp, Tcl_NewStringObj("tcl_rcFileName", -1), NULL,
    Tcl_NewStringObj("~/.tclshrc", -1), TCL_GLOBAL_ONLY);
#endif

    return TCL_OK;
}
