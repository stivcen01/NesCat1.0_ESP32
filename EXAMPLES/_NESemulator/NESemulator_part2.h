//MMCLIST.C:
// implemented mapper interfaces
const mapintf_t *mappers[] = {
  &map0_intf,
  &map1_intf,
  &map2_intf,
  &map3_intf,
  &map4_intf,
  &map5_intf,
  &map7_intf,
  &map8_intf,
  &map9_intf,
  &map11_intf,
  &map15_intf,
  &map16_intf,
  &map18_intf,
  &map19_intf,
  &map21_intf,
  &map22_intf,
  &map23_intf,
  &map24_intf,
  &map25_intf,
  &map32_intf,
  &map33_intf,
  &map34_intf,
  &map40_intf,
  &map64_intf,
  &map65_intf,
  &map66_intf,
  &map70_intf,
  &map75_intf,
  &map78_intf,
  &map79_intf,
  &map85_intf,
  &map94_intf,
  &map99_intf,
  &map231_intf,
  NULL
};


// Check to see if this mapper is supported
bool mmc_peek(int map_num)
{
  mapintf_t **map_ptr = (mapintf_t**)mappers;

  while (NULL != *map_ptr)
  {
    if ((*map_ptr)->number == map_num)
      return true;
    map_ptr++;
  }

  return false;
}

static void mmc_setpages(void)
{
  ///nes_log_printf("setting up mapper %d\n", mmc.intf->number);

  // Switch ROM into CPU space, set VROM/VRAM (done for ALL ROMs)
  mmc_bankrom(16, 0x8000, 0);
  mmc_bankrom(16, 0xC000, MMC_LASTBANK);
  mmc_bankvrom(8, 0x0000, 0);

  if (mmc.cart->flags & ROM_FLAG_FOURSCREEN)
  {
    ppu_mirror(0, 1, 2, 3);
  }
  else
  {
    if (MIRROR_VERT == mmc.cart->mirror)
      ppu_mirror(0, 1, 0, 1);
    else
      ppu_mirror(0, 0, 1, 1);
  }

  // if we have no VROM, switch in VRAM
  // TODO: fix this hack implementation
  if (0 == mmc.cart->vrom_banks) {
    ppu_setpage(8, 0, mmc.cart->vram);
    ppu_mirrorhipages();
  }
}


// Mapper initialization routine
void mmc_reset(void) {
  mmc_setpages();

  ppu_setlatchfunc(NULL);
  ppu_setvromswitch(NULL);

  if (mmc.intf->init) mmc.intf->init();
}

void mmc_destroy(mmc_t **nes_mmc);
void mmc_destroy(mmc_t **nes_mmc) {
  if (*nes_mmc)
    free(*nes_mmc);
}

mmc_t *mmc_temp;
mapintf_t **map_ptr;

mmc_t *mmc_create(rominfo_t *rominfo);
mmc_t *mmc_create(rominfo_t *rominfo) {


  for (map_ptr = (mapintf_t**)mappers; (*map_ptr)->number != rominfo->mapper_number; map_ptr++)  {
    if (NULL == *map_ptr) return NULL; // Should *never* happen
  }

  if (NULL == mmc_temp) mmc_temp = (mmc_t*)malloc(sizeof(mmc_t));
  if (NULL == mmc_temp) return NULL;

  memset(mmc_temp, 0, sizeof(mmc_t));

  mmc_temp->intf = *map_ptr;
  mmc_temp->cart = rominfo;

  mmc_setcontext(mmc_temp);
  return mmc_temp;
}


//--------------------------------------------------------------------------------
// Reset NES hardware
void nes_reset() {
  memset(nes.cpu->mem_page[0], 0, NES_RAMSIZE);
  if (nes.rominfo->vram) mem_trash(nes.rominfo->vram, 0x2000 * nes.rominfo->vram_banks);
  ///if (nes.rominfo->vram) memset(nes.rominfo->vram, 0, VRAM_LENGTH);
  ///if (nes.rominfo->vram) memset(nes.rominfo->vram, 0, 0x2000 * nes.rominfo->vram_banks);

  if (SOUND_ENABLED) apu_reset();
  ppu_reset();
  mmc_reset();
  nes6502_reset();

  nes.scanline = 241;
}

//--------------------------------------------------------------------------------
uint8_t nes_clearfiq(void) {
  if (nes.fiq_occurred) {
    nes.fiq_occurred = false;
    return 0x40;
  }
  return 0;
}

nes_t *machine;
sndinfo_t osd_sound;


// Initialize NES CPU, hardware, etc.
nes_t *nes_create(void);
nes_t *nes_create(void) {

  int i;

  Serial.println(">>> nes_create");
  MEMORY_STATUS();

  if (NULL == machine) machine = (nes_t*)malloc(sizeof(nes_t));
  if (NULL == machine) return NULL;

  MEMORY_STATUS();

  memset(machine, 0, sizeof(nes_t));

  ////machine->autoframeskip = true;

  // cpu
  if (NULL == machine->cpu) machine->cpu = (nes6502_context*) malloc(sizeof(nes6502_context));
  if (NULL == machine->cpu)  goto _fail;

  memset(machine->cpu, 0, sizeof(nes6502_context));

  // allocate 2kB RAM
  if (NULL == machine->cpu->mem_page[0]) machine->cpu->mem_page[0] = (uint8_t*)malloc(NES_RAMSIZE);
  if (NULL == machine->cpu->mem_page[0]) goto _fail;

  ///if (NULL != machine->cpu->mem_page[0]) memset(machine->cpu->mem_page[0], 0, sizeof(NES_RAMSIZE));

  // point all pages at NULL for now
  for (i = 1; i < NES6502_NUMBANKS; i++) machine->cpu->mem_page[i] = NULL;

  machine->cpu->read_handler = machine->readhandler;
  machine->cpu->write_handler = machine->writehandler;


  // apu
  if (SOUND_ENABLED) {
    osd_getsoundinfo(&osd_sound);
    ///machine->apu = apu_create(0, 25, NES_REFRESH_RATE, 256); //25fps
    machine->apu = apu_create(0, osd_sound.sample_rate, NES_REFRESH_RATE, osd_sound.bps);


    if (NULL == machine->apu) goto _fail;

    // set the IRQ routines
    machine->apu->irq_callback = nes_irq;
    machine->apu->irqclear_callback = nes_clearfiq;
  }

  // ppu
  machine->ppu = ppu_create();
  if (NULL == machine->ppu) goto _fail;

  return machine;

_fail:
  ///nes_destroy(&machine);
  return NULL;
}



static uint8_t ram_read(uint32_t address) {
  return nes.cpu->mem_page[0][address & (NES_RAMSIZE - 1)];
}

static void ram_write(uint32_t address, uint8_t value) {
  nes.cpu->mem_page[0][address & (NES_RAMSIZE - 1)] = value;
}

static void write_protect(uint32_t address, uint8_t value) {
  // don't allow write to go through
  UNUSED(address);
  UNUSED(value);
}

static uint8_t read_protect(uint32_t address) {
  // don't allow read to go through
  UNUSED(address);

  return 0xFF;
}

#define  LAST_MEMORY_HANDLER  { -1, -1, NULL }
// read/write handlers for standard NES
static nes6502_memread default_readhandler[] = {
  { 0x0800, 0x1FFF, ram_read },
  { 0x2000, 0x3FFF, ppu_read },
  { 0x4000, 0x4015, apu_read },
  { 0x4016, 0x4017, ppu_readhigh },
  LAST_MEMORY_HANDLER
};

static nes6502_memwrite default_writehandler[] = {
  { 0x0800, 0x1FFF, ram_write },
  { 0x2000, 0x3FFF, ppu_write },
  { 0x4000, 0x4013, apu_write },
  { 0x4015, 0x4015, apu_write },
  { 0x4014, 0x4017, ppu_writehigh },
  LAST_MEMORY_HANDLER
};

// this big nasty boy sets up the address handlers that the CPU uses
static void build_address_handlers(nes_t *machine);
static void build_address_handlers(nes_t *machine) {
  int count, num_handlers = 0;
  mapintf_t *intf;

  intf = machine->mmc->intf;

  memset(machine->readhandler, 0, sizeof(machine->readhandler));
  memset(machine->writehandler, 0, sizeof(machine->writehandler));

  for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++)  {
    if (NULL == default_readhandler[count].read_func)
      break;

    memcpy(&machine->readhandler[num_handlers], &default_readhandler[count], sizeof(nes6502_memread));
  }

  if (intf->sound_ext && SOUND_ENABLED) {
    if (NULL != intf->sound_ext->mem_read) {
      for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++) {
        if (NULL == intf->sound_ext->mem_read[count].read_func) break;
        memcpy(&machine->readhandler[num_handlers], &intf->sound_ext->mem_read[count], sizeof(nes6502_memread));
      }
    }
  }

  if (NULL != intf->mem_read) {
    for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++) {
      if (NULL == intf->mem_read[count].read_func) break;

      memcpy(&machine->readhandler[num_handlers], &intf->mem_read[count], sizeof(nes6502_memread));
    }
  }

  // TODO: poof! numbers
  machine->readhandler[num_handlers].min_range = 0x4018;
  machine->readhandler[num_handlers].max_range = 0x5FFF;
  machine->readhandler[num_handlers].read_func = read_protect;
  num_handlers++;
  machine->readhandler[num_handlers].min_range = -1;
  machine->readhandler[num_handlers].max_range = -1;
  machine->readhandler[num_handlers].read_func = NULL;
  num_handlers++;

  num_handlers = 0;

  for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++) {
    if (NULL == default_writehandler[count].write_func) break;

    memcpy(&machine->writehandler[num_handlers], &default_writehandler[count], sizeof(nes6502_memwrite));
  }

  if (intf->sound_ext) {
    if (NULL != intf->sound_ext->mem_write) {
      for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++) {
        if (NULL == intf->sound_ext->mem_write[count].write_func) break;
        memcpy(&machine->writehandler[num_handlers], &intf->sound_ext->mem_write[count], sizeof(nes6502_memwrite));
      }
    }
  }

  if (NULL != intf->mem_write) {
    for (count = 0; num_handlers < MAX_MEM_HANDLERS; count++, num_handlers++) {
      if (NULL == intf->mem_write[count].write_func) break;
      memcpy(&machine->writehandler[num_handlers], &intf->mem_write[count], sizeof(nes6502_memwrite));
    }
  }

  // catch-all for bad writes
  // TODO: poof! numbers
  machine->writehandler[num_handlers].min_range = 0x4018;
  machine->writehandler[num_handlers].max_range = 0x5FFF;
  machine->writehandler[num_handlers].write_func = write_protect;
  num_handlers++;
  machine->writehandler[num_handlers].min_range = 0x8000;
  machine->writehandler[num_handlers].max_range = 0xFFFF;
  machine->writehandler[num_handlers].write_func = write_protect;
  num_handlers++;
  machine->writehandler[num_handlers].min_range = -1;
  machine->writehandler[num_handlers].max_range = -1;
  machine->writehandler[num_handlers].write_func = NULL;
  num_handlers++;
}

void nes_setcontext(nes_t *machine);
void nes_setcontext(nes_t *machine) {
  if (SOUND_ENABLED) apu_setcontext(machine->apu);
  ppu_setcontext(machine->ppu);
  nes6502_setcontext(machine->cpu);
  mmc_setcontext(machine->mmc);

  nes = *machine;
}


void nes_checkfiq(int cycles) {
  nes.fiq_cycles -= cycles;
  if (nes.fiq_cycles <= 0) {
    nes.fiq_cycles += (int) NES_FIQ_PERIOD;
    if (0 == (nes.fiq_state & 0xC0)) {
      nes.fiq_occurred = true;
      nes6502_irq();
    }
  }
}


void nes_renderframe(bool draw_flag) {
  int elapsed_cycles;
  mapintf_t *mapintf = nes.mmc->intf;
  int in_vblank = 0;


  while (262 != nes.scanline) {

    ///if (NES_POWER==0) break;

    ppu_scanline(nes.scanline, draw_flag);
    if (241 == nes.scanline) {
      // 7-9 cycle delay between when VINT flag goes up and NMI is taken
      elapsed_cycles = nes6502_execute(7);
      nes.scanline_cycles -= elapsed_cycles;
      nes_checkfiq(elapsed_cycles);

      ppu_checknmi();

      if (mapintf->vblank) mapintf->vblank();
      in_vblank = 1;
    }

    if (mapintf->hblank) mapintf->hblank(in_vblank);

    nes.scanline_cycles = 0;

    nes.scanline_cycles += (float) NES_SCANLINE_CYCLES;
    elapsed_cycles = nes6502_execute((int) nes.scanline_cycles);
    nes.scanline_cycles -= (float) elapsed_cycles;
    nes_checkfiq(elapsed_cycles);

    ppu_endscanline(nes.scanline);
    nes.scanline++;
  }
  nes.scanline = 0;
}

//--------------------------------------------------------------------------------
void nes_destroy(nes_t **machine);
void nes_destroy(nes_t **machine)
{
  if (*machine)
  {
    ///    rom_free(&(*machine)->rominfo);
    mmc_destroy(&(*machine)->mmc);
    ppu_destroy(&(*machine)->ppu);
    apu_destroy(&(*machine)->apu);
    //      bmp_destroy(&(*machine)->vidbuf);
    if ((*machine)->cpu)
    {
      if ((*machine)->cpu->mem_page[0]) free((*machine)->cpu->mem_page[0]);
      free((*machine)->cpu);
    }

    free(*machine);
    *machine = NULL;
  }
}
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
