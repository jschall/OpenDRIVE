

/* Define memory regions. */
MEMORY
{
    rom (rx) :          ORIGIN = 0x08000000, LENGTH = 12K
    app (rx) :          ORIGIN = 0x08003000, LENGTH = 50K
    params (rx) :       ORIGIN = 0x0800F800, LENGTH =  2K
    ram (rwx) :         ORIGIN = 0x20000000, LENGTH = 16K-128
    app_bl_shared (rwx) : ORIGIN = 0x20000000+(16K-128), LENGTH = 128
}

/* Enforce emmition of the vector table. */
EXTERN (vector_table)

/* Define the entry point of the output file. */
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
    .text : {
        *(.vectors)	/* Vector table */
        *(.text*)	/* Program code */
        . = ALIGN(4);
        *(.rodata*)	/* Read-only data */
        . = ALIGN(4);
    } >rom

    .params (NOLOAD) : {
    } >params

    .app_bl_shared (NOLOAD) : {
    } >app_bl_shared

    .app (NOLOAD) : {
    } >app

    /* C++ Static constructors/destructors, also used for __attribute__
        * ((constructor)) and the likes */
    .preinit_array : {
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
    } >rom
    .init_array : {
        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } >rom
    .fini_array : {
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } >rom

    /*
        * Another section used by C++ stuff, appears when using newlib with
        * 64bit (long long) printf support
        */
    .ARM.extab : {
        *(.ARM.extab*)
    } >rom
    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >rom

    . = ALIGN(4);
    _etext = .;

    .data : {
        _data = .;
        *(.data*)	/* Read-write initialized data */
        . = ALIGN(4);
        _edata = .;
    } >ram AT >rom
    _data_loadaddr = LOADADDR(.data);

    .bss : {
        *(.bss*)	/* Read-write zero initialized data */
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >ram

    /*
        * The .eh_frame section appears to be used for C++ exception handling.
        * You may need to fix this if you're using C++.
        */
    /DISCARD/ : { *(.eh_frame) }

    . = ALIGN(4);
    end = .;
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram) - 8);

PROVIDE(_app_begin = ORIGIN(app));
PROVIDE(_app_end = ORIGIN(app) + LENGTH(app));

PROVIDE(_app_bl_shared_begin = ORIGIN(app_bl_shared));
PROVIDE(_app_bl_shared_end = ORIGIN(app_bl_shared) + LENGTH(app_bl_shared));

PROVIDE(_params_begin = ORIGIN(params));
PROVIDE(_params_end = ORIGIN(params) + LENGTH(params));
