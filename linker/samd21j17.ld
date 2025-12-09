/*
 * Copyright (c)  2024, Angus Logan <awjlogan@gmail.com>
 *                2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

PHDRS
{
  text PT_LOAD FLAGS(5); /* RX */
  data PT_LOAD FLAGS(6); /* RW */
}

MEMORY
{
  bootloader (rx) : ORIGIN = 0x00000000, LENGTH = 0x2000 /* 8K */
  flash (rx) : ORIGIN = 0x00002000, LENGTH = 0x1E000 /* 120k */
  ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3FFC /* 16k - 4 */
  blsm (rwx) : ORIGIN = 0x20003FFC, LENGTH = 0x4 /* 4 */
}

__top_flash = ORIGIN(flash) + LENGTH(flash);
__top_ram = ORIGIN(ram) + LENGTH(ram);

ENTRY(irq_handler_reset)

SECTIONS
{
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.vectors .vectors.*))
    *(.text .text.* .gnu.linkonce.t.*)
    *(.rodata .rodata* .gnu.linkonce.r.*)

    . = ALIGN(4);

    KEEP(*(.init))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;

    . = ALIGN(4);
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;

    . = ALIGN(4);
    KEEP(*(.fini))

    . = ALIGN(4);
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;

    . = ALIGN(4);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))

    . = ALIGN(4);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    . = ALIGN(4);
  } > flash : text

  .ARM.extab : {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > flash : text

  .ARM.exidx : {
    PROVIDE(__exidx_start = .);
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    PROVIDE(__exidx_end = .);
  } > flash : text

  .data :
  {
    . = ALIGN(4);
    _data = .;
    *(.ramfunc .ramfunc.*);
    *(.data .data.*);
    . = ALIGN(4);
    _edata = .;
  } > ram AT> flash : data

  _etext = LOADADDR(.data);

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _bss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > ram

  PROVIDE(_stack_top = __top_ram - 0);

  .blsm (NOLOAD) :
  {
    PROVIDE(__blsm = .);
    KEEP(*(.blsm))
  } > blsm
}
