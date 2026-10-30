/*
 *  PowerVR PCX2 (NEC/VideoLogic, e.g. Matrox m3D) 3D accelerator emulation.
 *
 *  Copyright (C) 2026 Zvi Rosen and contributors.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *  Port of the PCX2 device written for 86Box (github.com/z2k-retro/86Box,
 *  branch pcx-perf). The PCX2 is a PCI bus-master that has no display of
 *  its own: it reads its parameter lists from system RAM through a 128-entry
 *  page table, renders in tiles, and writes the finished pixels into the 2D
 *  card's linear frame buffer (SOFADDR). Here the whole frame is rendered
 *  when STARTRENDER is written; end of render (INTSTATUS, the interrupt) is
 *  reported a configurable time later, as the card would.
 *
 *  The renderer itself (pcx2/vid_powervr_pcx_render.c) is MIT-licensed.
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <vector>

#include "dosbox.h"
#include "config.h"
#include "logging.h"
#include "setup.h"
#include "control.h"
#include "mem.h"
#include "paging.h"
#include "pic.h"
#include "pci_bus.h"

extern "C" {
#include "pcx2/vid_powervr_pcx_render.h"
#include "pcx2/vid_powervr_pcx_tables.h"
}

Bits RegisterPCIDevice(PCI_Device* device, Bits bus, Bits slot);
bool UnregisterPCIDevice(PCI_Device* device);

#define PCX_VENDOR_NEC      0x1033
#define PCX2_DEVICE_ID      0x0046
#define PCX_BAR0_SIZE       0x00010000u /* register file */
#define PCX_BAR1_SIZE       0x00400000u /* 4 MB texture memory */
#define PCX_TEXMEM_SIZE     0x00400000u

#define PCX_ID              0x000
#define PCX_REVISION        0x001
#define PCX_SOFTRESET       0x002
#define PCX_INTSTATUS       0x003
#define PCX_INTMASK         0x004
#define PCX_STARTRENDER     0x005
#define PCX_FOGAMOUNT       0x006
#define PCX_OBJECT_OFFSET   0x007
#define PCX_PAGE_CTRL       0x008
#define PCX_TMEM_SETUP      0x00c
#define PCX_TMEM_REFRESH    0x00d
#define PCX_FOGCOL          0x00e
#define PCX_CAMERA          0x00f
#define PCX_PACKMODE        0x010
#define PCX_LSTRIDE         0x012
#define PCX_SOFADDR         0x013
#define PCX_XCLIP           0x014
#define PCX_GPPORT          0x016
#define PCX_BILINEAR        0x019
#define PCX_FASTFOG         0x01d
#define PCX_FOG_TABLE       0x080
#define PCX_TLB             0x100
#define PCX_TLB_ENTRIES     128
#define PCX_TLB_MASK        0x00000fffu
#define PCX_DIVIDER_TABLE   0x200
#define PCX_REG_COUNT       0x400

#define PCX_INT_FRAME_DONE  0x00000007u

static const struct {
    uint16_t idx;
    uint32_t mask;
} pcx2_write_masks[] = {
    { 0x06, 0x0000001f }, { 0x07, 0xfffffffd }, { 0x08, 0x000003ff }, { 0x09, 0x00000000 },
    { 0x0a, 0x000fffff }, { 0x0b, 0x000fffff }, { 0x0e, 0x01ffffff }, { 0x0f, 0x000007ff },
    { 0x10, 0x000000ff }, { 0x12, 0x00001ffc }, { 0x13, 0xfffffffc }, { 0x14, 0x13ff13ff },
    { 0x15, 0x00000000 }, { 0x17, 0x00000000 }, { 0x18, 0x0000000f }, { 0x19, 0x00000007 },
    { 0x1a, 0x00000000 }, { 0x1d, 0x000001ff },
};

class PCX2;
static PCX2 *pcx2_dev = NULL;

/* ---------------------------------------------------------------------- */

class PCX2_RegHandler : public PageHandler {
public:
    PCX2_RegHandler() : PageHandler(PFLAG_NOCODE) {}
    uint8_t readb(PhysPt addr) override;
    uint16_t readw(PhysPt addr) override;
    uint32_t readd(PhysPt addr) override;
    void writeb(PhysPt addr, uint8_t val) override;
    void writew(PhysPt addr, uint16_t val) override;
    void writed(PhysPt addr, uint32_t val) override;
};

class PCX2_TexHandler : public PageHandler {
public:
    PCX2_TexHandler() : PageHandler(PFLAG_READABLE | PFLAG_WRITEABLE | PFLAG_NOCODE) {}
    HostPt GetHostReadPt(PageNum phys_page) override;
    HostPt GetHostWritePt(PageNum phys_page) override { return GetHostReadPt(phys_page); }
};

static PCX2_RegHandler pcx2_reg_handler;
static PCX2_TexHandler pcx2_tex_handler;

static MEM_Callout_t pcx2_cb_regs = MEM_Callout_t_none;
static MEM_Callout_t pcx2_cb_tex  = MEM_Callout_t_none;

static PageHandler *pcx2_regs_cb(MEM_CalloutObject &co, Bitu phys_page) {
    (void)co; (void)phys_page;
    return &pcx2_reg_handler;
}
static PageHandler *pcx2_tex_cb(MEM_CalloutObject &co, Bitu phys_page) {
    (void)co; (void)phys_page;
    return &pcx2_tex_handler;
}

static void pcx2_end_of_render(Bitu val);

class PCX2 {
public:
    uint32_t  regs[PCX_REG_COUNT];
    uint8_t  *tex_mem;
    uint32_t *isp_buf;
    uint32_t *rgb;
    pcxr_job_t *rjob;

    uint32_t bar0, bar1;
    bool     mem_on, bm_on;
    uint32_t int_status;
    uint32_t tlb_hiwater;
    bool     render_busy;
    bool     irq_raised;
    unsigned irq;
    unsigned threads;
    double   render_delay_ms;
    uint32_t frames;
    uint32_t frames_skipped;

    PCX2(unsigned irq_, unsigned threads_, double delay_ms) {
        irq = irq_;
        threads = threads_;
        render_delay_ms = delay_ms;
        tex_mem = (uint8_t *)calloc(1, PCX_TEXMEM_SIZE);
        isp_buf = (uint32_t *)calloc(PCX_TLB_ENTRIES, 0x4000);
        rgb = (uint32_t *)calloc((size_t)PCXR_MAX_W * PCXR_MAX_H, sizeof(uint32_t));
        rjob = pcxr_job_create();
        bar0 = bar1 = 0;
        mem_on = bm_on = false;
        frames = frames_skipped = 0;
        Reset();
    }

    ~PCX2() {
        PIC_RemoveEvents(pcx2_end_of_render);
        SetIRQ(false);
        if (rjob) pcxr_job_destroy(rjob);
        free(rgb);
        free(isp_buf);
        free(tex_mem);
    }

    void Reset() {
        memset(regs, 0, sizeof(regs));
        regs[PCX_ID]           = ((uint32_t)PCX_VENDOR_NEC << 16) | PCX2_DEVICE_ID;
        regs[PCX_REVISION]     = 1;
        regs[PCX_TMEM_SETUP]   = 0x377777ff;
        regs[PCX_TMEM_REFRESH] = 0x00000700;
        regs[PCX_GPPORT]       = 0x0000f616;
        memcpy(&regs[PCX_FOG_TABLE], pcx2_fog_rom, sizeof(pcx2_fog_rom));
        memcpy(&regs[PCX_DIVIDER_TABLE], pcx2_divider_rom, sizeof(pcx2_divider_rom));
        int_status = 0;
        tlb_hiwater = 0;
        render_busy = false;
        irq_raised = false;
    }

    void SetIRQ(bool on) {
        if (irq == 0 || irq > 15) return;
        if (on && !irq_raised) {
            PIC_ActivateIRQ(irq);
            irq_raised = true;
        } else if (!on && irq_raised) {
            PIC_DeActivateIRQ(irq);
            irq_raised = false;
        }
    }

    void UpdateIRQ() {
        SetIRQ((int_status & regs[PCX_INTMASK]) != 0);
    }

    uint32_t PageBytes() const {
        switch ((regs[PCX_PAGE_CTRL] >> 8) & 3) {
            case 0:  return 0x1000;
            case 1:  return 0x2000;
            default: return 0x4000;
        }
    }

    /* Copy count bytes of guest physical memory. System RAM is read
       directly; anything else (another device's memory) goes through its
       page handler. */
    void ReadPhys(uint32_t pa, uint8_t *dst, uint32_t count) {
        if ((uint64_t)pa + count <= (uint64_t)MEM_TotalPages() * 4096u) {
            memcpy(dst, MemBase + pa, count);
            return;
        }
        while (count) {
            const Bitu page = pa >> 12;
            const uint32_t off = pa & 0xfff;
            uint32_t n = 0x1000 - off;
            if (n > count) n = count;
            PageHandler *ph = MEM_GetPageHandler(page);
            if (ph && (ph->getFlags() & PFLAG_READABLE)) {
                memcpy(dst, ph->GetHostReadPt(page) + off, n);
            } else {
                memset(dst, 0xff, n);
            }
            pa += n; dst += n; count -= n;
        }
    }

    /* Bus-master write into guest physical memory: normally the VESA
       linear frame buffer of the 2D card. */
    void WritePhys(uint32_t pa, const uint8_t *src, uint32_t count) {
        while (count) {
            const Bitu page = pa >> 12;
            const uint32_t off = pa & 0xfff;
            uint32_t n = 0x1000 - off;
            if (n > count) n = count;
            if ((uint64_t)pa + n <= (uint64_t)MEM_TotalPages() * 4096u) {
                memcpy(MemBase + pa, src, n);
            } else {
                PageHandler *ph = MEM_GetPageHandler(page);
                if (ph && (ph->getFlags() & PFLAG_WRITEABLE)) {
                    memcpy(ph->GetHostWritePt(page) + off, src, n);
                }
                /* Pages without host memory behind them are not written:
                   their handlers expect CPU (linear) addresses. */
            }
            pa += n; src += n; count -= n;
        }
    }

    void RenderFrame() {
        const uint32_t page_bytes = PageBytes();
        uint32_t entries = tlb_hiwater ? tlb_hiwater : PCX_TLB_ENTRIES;
        if (entries > PCX_TLB_ENTRIES) entries = PCX_TLB_ENTRIES;

        for (uint32_t i = 0; i < entries; i++) {
            const uint32_t pa = ((regs[PCX_PAGE_CTRL] & 0xff) << 24) | ((regs[PCX_TLB + i] & PCX_TLB_MASK) << 12);
            ReadPhys(pa, (uint8_t *)isp_buf + (size_t)i * page_bytes, page_bytes);
        }

        pcxr_frame_t job;
        pcxr_result_t res;
        uint32_t fog[128];
        memcpy(fog, &regs[PCX_FOG_TABLE], sizeof(fog));
        memset(&job, 0, sizeof(job));
        memset(&res, 0, sizeof(res));
        job.isp           = isp_buf;
        job.isp_words     = entries * page_bytes / 4;
        job.tex           = (const uint32_t *)tex_mem;
        job.tex_words     = PCX_TEXMEM_SIZE / 4;
        job.object_offset = regs[PCX_OBJECT_OFFSET];
        job.fog_shift     = regs[PCX_FOGAMOUNT];
        job.fog_colour    = regs[PCX_FOGCOL];
        job.camera        = regs[PCX_CAMERA];
        job.xclip         = regs[PCX_XCLIP];
        job.bilinear      = regs[PCX_BILINEAR];
        job.fastfog       = regs[PCX_FASTFOG];
        job.fog_table     = fog;

        memset(rgb, 0, (size_t)PCXR_MAX_W * PCXR_MAX_H * sizeof(uint32_t));
        res.max_x = res.max_y = -1;
        if (pcxr_prepare(rjob, &job, &res) > 0) {
            std::vector<std::thread> helpers;
            for (unsigned i = 1; i < threads; i++)
                helpers.emplace_back([this]() { pcxr_render_tiles(rjob, rgb, 0, 0); });
            pcxr_render_tiles(rjob, rgb, 0, 0);
            for (auto &t : helpers) t.join();
        }

        if (res.error && frames_skipped++ < 10)
            LOG_MSG("PCX2: frame %u: malformed tile list (error %d)", frames, res.error);

        WriteOutput(res);
    }

    /* Only pixels the chip wrote go out; the rest of the surface stays as
       it was, as on the card. */
    void WriteOutput(const pcxr_result_t &r) {
        const uint32_t sofaddr = regs[PCX_SOFADDR];
        const uint32_t stride = regs[PCX_LSTRIDE];
        const uint32_t packmode = regs[PCX_PACKMODE];
        const uint32_t xc = regs[PCX_XCLIP];
        const int bpp = ((packmode & 3) == 0) ? 4 : ((packmode & 3) == 1) ? 3 : 2;
        int x0 = r.min_x, x1 = r.max_x;
        uint8_t row[PCXR_MAX_W * 4];

        if (!sofaddr || r.error || r.max_x < 0 || r.max_y < 0)
            return;
        if ((xc & (1u << 12)) && x0 < (int)(xc & 0x3ff) - 1)
            x0 = (int)(xc & 0x3ff) - 1;
        if ((xc & (1u << 28)) && x1 > (int)((xc >> 16) & 0x3ff) - 1)
            x1 = (int)((xc >> 16) & 0x3ff) - 1;
        if (x1 >= PCXR_MAX_W)
            x1 = PCXR_MAX_W - 1;
        if (x0 < 0) x0 = 0;

        for (int y = r.min_y; y <= r.max_y && y < PCXR_MAX_H; y++) {
            const uint32_t *src = &rgb[y * PCXR_MAX_W];
            int x = x0;
            while (x <= x1) {
                while (x <= x1 && !(src[x] & PCXR_WRITTEN))
                    x++;
                if (x > x1)
                    break;
                const int start = x;
                while (x <= x1 && (src[x] & PCXR_WRITTEN))
                    x++;
                const uint32_t len = pcxr_pack_run(packmode, src, start, x - start, y, row);
                WritePhys(sofaddr + (uint32_t)y * stride + (uint32_t)start * bpp, row, len);
            }
        }
    }

    void StartRender() {
        frames++;
        if (render_busy) {
            /* The previous frame's end of render has not been reported
               yet; report it now. */
            PIC_RemoveEvents(pcx2_end_of_render);
            EndOfRender();
        }
        if (frames == 1)
            LOG_MSG("PCX2: first frame: SOFADDR=%08x LSTRIDE=%u PACKMODE=%02x XCLIP=%08x PAGE_CTRL=%08x TLB=%u",
                    regs[PCX_SOFADDR], regs[PCX_LSTRIDE], regs[PCX_PACKMODE], regs[PCX_XCLIP],
                    regs[PCX_PAGE_CTRL], tlb_hiwater);
        RenderFrame();
        render_busy = true;
        if (render_delay_ms > 0.0)
            PIC_AddEvent(pcx2_end_of_render, render_delay_ms);
        else
            EndOfRender();
    }

    void EndOfRender() {
        render_busy = false;
        int_status |= PCX_INT_FRAME_DONE;
        UpdateIRQ();
    }

    uint32_t RegRead(uint32_t idx) {
        switch (idx) {
            case PCX_GPPORT:
                return 0x0000f616;
            case PCX_INTSTATUS: {
                const uint32_t v = int_status;
                if (v) {
                    int_status = 0;
                    UpdateIRQ();
                }
                return v;
            }
            default:
                return regs[idx];
        }
    }

    void RegWrite(uint32_t idx, uint32_t val) {
        if (idx >= PCX_FOG_TABLE && idx < PCX_TLB)
            return; /* ROM */
        if (idx >= PCX_TLB + PCX_TLB_ENTRIES)
            return; /* unimplemented TLB entries and the divider ROM */
        if (idx >= PCX_TLB)
            val &= PCX_TLB_MASK;
        for (unsigned i = 0; i < sizeof(pcx2_write_masks) / sizeof(pcx2_write_masks[0]); i++)
            if (pcx2_write_masks[i].idx == idx)
                val &= pcx2_write_masks[i].mask;

        switch (idx) {
            case PCX_ID:
            case PCX_REVISION:
            case PCX_INTSTATUS:
                break;
            case PCX_SOFTRESET:
                regs[idx] = val;
                if ((val & 1) && render_busy) {
                    PIC_RemoveEvents(pcx2_end_of_render);
                    render_busy = false;
                }
                break;
            case PCX_INTMASK:
                regs[idx] = val;
                UpdateIRQ();
                break;
            case PCX_STARTRENDER:
                regs[idx] = val;
                StartRender();
                break;
            default:
                regs[idx] = val;
                if (idx >= PCX_TLB && idx - PCX_TLB + 1 > tlb_hiwater)
                    tlb_hiwater = idx - PCX_TLB + 1;
                break;
        }
    }

    void RemapBARs() {
        MEM_CalloutObject *cb;

        cb = MEM_GetCallout(pcx2_cb_regs);
        cb->Uninstall();
        if (mem_on && bar0)
            cb->Install(bar0 >> 12, MEMMASK_Combine(MEMMASK_FULL, MEMMASK_Range(PCX_BAR0_SIZE >> 12)), pcx2_regs_cb);
        MEM_PutCallout(cb);

        cb = MEM_GetCallout(pcx2_cb_tex);
        cb->Uninstall();
        if (mem_on && bar1)
            cb->Install(bar1 >> 12, MEMMASK_Combine(MEMMASK_FULL, MEMMASK_Range(PCX_BAR1_SIZE >> 12)), pcx2_tex_cb);
        MEM_PutCallout(cb);

        PAGING_ClearTLB();
    }
};

static void pcx2_end_of_render(Bitu val) {
    (void)val;
    if (pcx2_dev) pcx2_dev->EndOfRender();
}

uint32_t PCX2_RegHandler::readd(PhysPt addr) {
    addr = PAGING_GetPhysicalAddress(addr);
    if (!pcx2_dev) return 0xffffffff;
    const uint32_t off = (addr - pcx2_dev->bar0) & (PCX_BAR0_SIZE - 1);
    if (off >= PCX_REG_COUNT * 4) return 0xffffffff;
    return pcx2_dev->RegRead(off >> 2);
}
uint16_t PCX2_RegHandler::readw(PhysPt addr) {
    return (uint16_t)(readd(addr & ~3u) >> ((addr & 2) * 8));
}
uint8_t PCX2_RegHandler::readb(PhysPt addr) {
    return (uint8_t)(readd(addr & ~3u) >> ((addr & 3) * 8));
}
void PCX2_RegHandler::writed(PhysPt addr, uint32_t val) {
    addr = PAGING_GetPhysicalAddress(addr);
    if (!pcx2_dev) return;
    const uint32_t off = (addr - pcx2_dev->bar0) & (PCX_BAR0_SIZE - 1);
    if (off >= PCX_REG_COUNT * 4) return;
    pcx2_dev->RegWrite(off >> 2, val);
}
void PCX2_RegHandler::writew(PhysPt addr, uint16_t val) {
    const PhysPt pa = PAGING_GetPhysicalAddress(addr);
    if (!pcx2_dev) return;
    const uint32_t off = (pa - pcx2_dev->bar0) & (PCX_BAR0_SIZE - 1);
    if (off >= PCX_REG_COUNT * 4) return;
    const uint32_t idx = off >> 2, shift = (off & 2) * 8;
    pcx2_dev->RegWrite(idx, (pcx2_dev->regs[idx] & ~(0xffffu << shift)) | ((uint32_t)val << shift));
}
void PCX2_RegHandler::writeb(PhysPt addr, uint8_t val) {
    const PhysPt pa = PAGING_GetPhysicalAddress(addr);
    if (!pcx2_dev) return;
    const uint32_t off = (pa - pcx2_dev->bar0) & (PCX_BAR0_SIZE - 1);
    if (off >= PCX_REG_COUNT * 4) return;
    const uint32_t idx = off >> 2, shift = (off & 3) * 8;
    pcx2_dev->RegWrite(idx, (pcx2_dev->regs[idx] & ~(0xffu << shift)) | ((uint32_t)val << shift));
}

HostPt PCX2_TexHandler::GetHostReadPt(PageNum phys_page) {
    static uint8_t dummy[4096];
    if (!pcx2_dev) return dummy;
    const uint32_t page = (uint32_t)(phys_page - (pcx2_dev->bar1 >> 12)) & ((PCX_BAR1_SIZE >> 12) - 1);
    return pcx2_dev->tex_mem + (size_t)page * 4096u;
}

/* ---------------------------------------------------------------------- */
/* PCI configuration space                                                 */
/* ---------------------------------------------------------------------- */

class PCI_PCX2Device : public PCI_Device {
public:
    PCI_PCX2Device(uint32_t bar0, uint32_t bar1, unsigned irq) : PCI_Device(PCX_VENDOR_NEC, PCX2_DEVICE_ID) {
        config[0x04] = 0x06;    /* memory space and bus mastering on */
        config[0x05] = 0x00;
        config[0x06] = 0xa0;    /* status 02a0: 66 MHz capable, fast back-to-back */
        config[0x07] = 0x02;    /* medium DEVSEL */
        config[0x08] = 0x01;    /* revision */
        config[0x09] = 0x00;
        config[0x0a] = 0x80;    /* other multimedia device */
        config[0x0b] = 0x04;
        config[0x0e] = 0x00;
        config[0x3c] = (irq >= 1 && irq <= 15) ? (unsigned char)irq : 0xff;
        config[0x3d] = 0x01;    /* INTA */

        host_writew(config_writemask + 0x04, 0x0146); /* memory, bus master, parity, SERR */
        config_writemask[0x0c] = 0xff;
        config_writemask[0x0d] = 0xff;
        config_writemask[0x3c] = 0xff;
        host_writed(config_writemask + 0x10, ~(PCX_BAR0_SIZE - 1));
        host_writed(config_writemask + 0x14, ~(PCX_BAR1_SIZE - 1));
        host_writed(config + 0x10, (bar0 & ~(PCX_BAR0_SIZE - 1)) | 0x8);
        host_writed(config + 0x14, (bar1 & ~(PCX_BAR1_SIZE - 1)) | 0x8);
    }

    void Apply() {
        if (!pcx2_dev) return;
        pcx2_dev->bar0   = host_readd(config + 0x10) & ~(PCX_BAR0_SIZE - 1);
        pcx2_dev->bar1   = host_readd(config + 0x14) & ~(PCX_BAR1_SIZE - 1);
        pcx2_dev->mem_on = (config[0x04] & 0x02) != 0;
        pcx2_dev->bm_on  = (config[0x04] & 0x04) != 0;
        pcx2_dev->RemapBARs();
    }

    void config_write(uint8_t regnum, Bitu iolen, uint32_t value) override {
        if (iolen == 1) {
            if (regnum == 0x07) { /* status: write 1 to clear */
                config[regnum] &= (unsigned char)~(value & 0xf9);
                return;
            }
            const unsigned char mask = config_writemask[regnum];
            config[regnum] = (unsigned char)((config[regnum] & ~mask) | ((unsigned char)value & mask));
            if (regnum == 0x04 || (regnum >= 0x10 && regnum <= 0x17))
                Apply();
        } else {
            PCI_Device::config_write(regnum, iolen, value);
        }
    }
};

static PCI_PCX2Device *pcx2_pci = NULL;

/* ---------------------------------------------------------------------- */

static void PCX2_Destroy(Section *sec) {
    (void)sec;
    if (pcx2_pci) {
        UnregisterPCIDevice(pcx2_pci);
        delete pcx2_pci;
        pcx2_pci = NULL;
    }
    if (pcx2_dev) {
        if (pcx2_cb_regs != MEM_Callout_t_none) {
            MEM_CalloutObject *cb = MEM_GetCallout(pcx2_cb_regs);
            cb->Uninstall();
            MEM_PutCallout(cb);
        }
        if (pcx2_cb_tex != MEM_Callout_t_none) {
            MEM_CalloutObject *cb = MEM_GetCallout(pcx2_cb_tex);
            cb->Uninstall();
            MEM_PutCallout(cb);
        }
        delete pcx2_dev;
        pcx2_dev = NULL;
    }
}

static uint32_t pcx2_assigned = 0;

static void PCX2_OnPowerOn(Section *sec) {
    (void)sec;
    PCX2_Destroy(NULL);

    Section_prop *section = static_cast<Section_prop *>(control->GetSection("powervr"));
    if (section == NULL || !section->Get_bool("pcx2"))
        return;
    if (!pcibus_enable) {
        LOG_MSG("PCX2: needs the PCI bus (enable pci bus=true in [dosbox])");
        return;
    }

    int irq = section->Get_int("pcx2 irq");
    int threads = section->Get_int("pcx2 render threads");
    double delay = section->Get_double("pcx2 render delay");
    if (threads <= 0) {
        threads = (int)std::thread::hardware_concurrency();
        if (threads > 8) threads = 8;
        if (threads < 1) threads = 1;
    }
    if (threads > 16) threads = 16;
    if (delay < 0) delay = 0;

    if (pcx2_cb_regs == MEM_Callout_t_none) {
        pcx2_cb_regs = MEM_AllocateCallout(MEM_TYPE_PCI);
        pcx2_cb_tex  = MEM_AllocateCallout(MEM_TYPE_PCI);
        if (pcx2_cb_regs == MEM_Callout_t_none || pcx2_cb_tex == MEM_Callout_t_none)
            E_Exit("PCX2: unable to allocate memory callouts");
    }

    /* BAR1 (4 MB) at the start of an 8 MB block above RAM, BAR0 after it. */
    uint32_t base = pcx2_assigned;
    const uint32_t ram_top = (uint32_t)(MEM_TotalPages() * 4096u);
    if (base < ram_top) base = ram_top;
    base = (base + 0x7fffffu) & ~0x7fffffu;

    pcx2_dev = new PCX2((unsigned)irq, (unsigned)threads, delay);
    pcx2_pci = new PCI_PCX2Device(base + PCX_BAR1_SIZE, base, (unsigned)irq);
    RegisterPCIDevice(pcx2_pci, -1, -1);
    pcx2_pci->Apply();

    LOG_MSG("PCX2: PowerVR PCX2 (1033:0046) registers at %08x, texture memory at %08x, IRQ %d, %d render thread(s)",
            pcx2_dev->bar0, pcx2_dev->bar1, irq, threads);
}

void PCX2_Init() {
    pcx2_assigned = MEM_HardwareAllocate("PowerVR PCX2", 8ul << 20ul);
    AddExitFunction(AddExitFunctionFuncPair(PCX2_Destroy), true);
    AddVMEventFunction(VM_EVENT_POWERON, AddVMEventFunctionFuncPair(PCX2_OnPowerOn));
}
