/*
 * QEMU Busmouse emulation
 * 
 * Copyright (c) 2005 Michael Engel (engel-at-informatik.uni-marburg.de)
 * using hints from parallel.c and ps2.c
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to dea
l
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "vl.h"

/*
 * These are the definitions for the busmouse controller registers
 * and internal state
 */

struct BusmouseState {
    uint8_t data;
    uint8_t signature;
    uint8_t control;
    uint8_t interrupt;
    uint8_t config;
    uint8_t command;
    int irq;
    int irq_pending;
    CharDriverState *chr;
    int hw_driver;
    uint16_t mouse_dx;
    uint16_t mouse_dy;
    uint16_t mouse_dz;
    uint16_t mouse_buttons;
};

static void busmouse_update_irq(BusmouseState *s);

static void busmouse_event(void *opaque, 
                            int dx, int dy, int dz, int buttons_state)
{
    BusmouseState *s = opaque;

    s->mouse_dx += dx;
    s->mouse_dy += dy;
    s->mouse_dz += dz;
    /* XXX: SDL sometimes generates nul events: we delete them */
    if (s->mouse_dx == 0 && s->mouse_dy == 0 && s->mouse_dz == 0 &&
        s->mouse_buttons == buttons_state)
       return;
    s->mouse_buttons = buttons_state;
    
    s->irq_pending = 1;
    busmouse_update_irq(s); 
}

static void busmouse_update_irq(BusmouseState *s)
{
    if (s->irq_pending)
        pic_set_irq(s->irq, 1);
    else
        pic_set_irq(s->irq, 0);
}

static void busmouse_ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    BusmouseState *s = opaque;
    
    addr &= 0xf;
    switch(addr) {
    case 0xc: /* data port */
        break;
    case 0xd: /* signature port */
        break;
    case 0xe: /* control/interrupt port */
        s->command = val;
        break;
    case 0xf: /* config port */
        break;
    }
}

static uint32_t busmouse_ioport_read(void *opaque, uint32_t addr)
{
    BusmouseState *s = opaque;
    uint32_t ret = 0x00;
    static int interrupt_val = 0x01;

    addr &= 0xf;
    switch(addr) {
    case 0xc: /* data port */
        s->irq_pending = 0;
        switch (s->command) {
        case 0x00: /* no op? */
            break;
        case 0x80: /* x low */
            ret = s->mouse_dx & 0xf;
            ret += (7 - s->mouse_buttons) << 5; /* button state */
            break;
        case 0xa0: /* x high */
            ret = (s->mouse_dx >> 4) & 0xf;
            s->mouse_dx = 0;
            break;
        case 0xc0: /* y low */
            ret = s->mouse_dy & 0xf;
            break;
        case 0xe0: /* y high */
            ret =  (s->mouse_dy >> 4) & 0xf;
            s->mouse_dy = 0;
            break;
        }
        busmouse_update_irq(s);
        break;
    case 0xd: /* signature port */
        ret = 0xa5; /* return signature byte */
        busmouse_update_irq(s);
        break;
    case 0xe: /* control/interrupt port */
        ret = interrupt_val;
        interrupt_val = (interrupt_val << 1) & 0xff;
        if (interrupt_val == 0) interrupt_val = 1;

        break;
    case 0xf: /* config port */
        break;
    }
    return ret;
}

/* If fd is zero, it means that the busmouse device uses the console */
BusmouseState *busmouse_init(int base, int irq, CharDriverState *chr)
{
    BusmouseState *s;

    s = qemu_mallocz(sizeof(BusmouseState));
    if (!s)
        return NULL;
    s->chr = chr;
    s->hw_driver = 0;
    s->irq = irq;
    s->data = 0;
    s->mouse_buttons = 0x0;
    register_ioport_write(base, 8, 1, busmouse_ioport_write, s);
    register_ioport_read(base, 8, 1, busmouse_ioport_read, s);
    qemu_add_mouse_event_handler(busmouse_event, s,0,"QEMU Busmouse");

    return s;
}
