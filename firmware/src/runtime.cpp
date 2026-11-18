// Bits of C runtime the firmware provides itself.
//
// We link with -nostartfiles (ST's startup file is the entry point), which
// also drops crti.o. newlib's __libc_init_array, called from the startup
// code to run C++ static constructors, still calls _init(), so it needs to
// exist. Constructors themselves run from .init_array; _init has nothing to do.
extern "C" void _init() {}
extern "C" void _fini() {}
