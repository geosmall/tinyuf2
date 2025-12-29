#include "board_api.h"
#include "stm32h7xx_hal.h"

#ifndef BOARD_PFLASH_EN
#include "qspi_status.h"

#ifdef W25Qx_SPI
#include "components/w25qxx/w25qxx.h"
#endif // W25Qx_SPI

#ifdef W25Qx_QSPI
#include "components/w25qxx/w25qxx_qspi.h"
#endif // W25Qx_QSPI

#ifdef IS25LP064A
#include "components/is25lp064a/is25lp064a_qspi.h"
#include "components/is25lp064a/is25lp064a.h"
#endif

#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
QSPI_HandleTypeDef _qspi_flash;
#endif // BOARD_QSPI_FLASH_EN

#if defined (BOARD_SPI_FLASH_EN) && (BOARD_SPI_FLASH_EN == 1)
SPI_HandleTypeDef _spi_flash;
#endif // BOARD_SPI_FLASH_EN

#endif // !BOARD_PFLASH_EN

//--------------------------------------------------------------------+
// H743 Internal Flash Support (2MB dual-bank, 128KB sectors)
//--------------------------------------------------------------------+
#ifdef BOARD_PFLASH_EN

#define FLASH_BASE_ADDR         0x08000000UL

// LED error indication: rapid blink pattern to signal flash failure
// Visible feedback for users without UART connection
static void flash_error_blink(void)
{
  for (int i = 0; i < 10; i++)
  {
    board_led_write(0xff);
    HAL_Delay(50);
    board_led_write(0x00);
    HAL_Delay(50);
  }
}
#define H743_SECTOR_SIZE        (128 * 1024)  // 128KB per sector
#define H743_BANK1_SECTORS      8
#define H743_BANK2_SECTORS      8
#define H743_TOTAL_SECTORS      (H743_BANK1_SECTORS + H743_BANK2_SECTORS)

// Bootloader occupies first sector (128KB), protect it
#define BOOTLOADER_SECTOR_COUNT 1

static uint8_t erased_sectors[H743_TOTAL_SECTORS] = { 0 };

static bool is_blank(uint32_t addr, uint32_t size)
{
  for (uint32_t i = 0; i < size; i += sizeof(uint32_t))
  {
    if (*(uint32_t*)(addr + i) != 0xFFFFFFFF)
    {
      return false;
    }
  }
  return true;
}

static uint32_t get_sector_number(uint32_t addr)
{
  // H743: All sectors are 128KB
  // Bank 1: sectors 0-7 (0x08000000 - 0x080FFFFF)
  // Bank 2: sectors 0-7 (0x08100000 - 0x081FFFFF)
  uint32_t offset = addr - FLASH_BASE_ADDR;

  if (offset < (H743_BANK1_SECTORS * H743_SECTOR_SIZE))
  {
    // Bank 1
    return offset / H743_SECTOR_SIZE;
  }
  else
  {
    // Bank 2 - sectors numbered 0-7 within bank
    return (offset - (H743_BANK1_SECTORS * H743_SECTOR_SIZE)) / H743_SECTOR_SIZE;
  }
}

static uint32_t get_bank_number(uint32_t addr)
{
  uint32_t offset = addr - FLASH_BASE_ADDR;
  return (offset < (H743_BANK1_SECTORS * H743_SECTOR_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
}

static uint32_t get_sector_addr(uint32_t sector, uint32_t bank)
{
  if (bank == FLASH_BANK_1)
  {
    return FLASH_BASE_ADDR + (sector * H743_SECTOR_SIZE);
  }
  else
  {
    return FLASH_BASE_ADDR + (H743_BANK1_SECTORS * H743_SECTOR_SIZE) + (sector * H743_SECTOR_SIZE);
  }
}

static bool flash_erase(uint32_t addr)
{
  uint32_t sector = get_sector_number(addr);
  uint32_t bank = get_bank_number(addr);
  uint32_t sector_addr = get_sector_addr(sector, bank);

  // Global sector index for tracking
  uint32_t global_sector = (bank == FLASH_BANK_1) ? sector : (H743_BANK1_SECTORS + sector);

  // Already erased?
  if (erased_sectors[global_sector])
  {
    return true;
  }

  // Mark as erased
  erased_sectors[global_sector] = 1;

  // Check if already blank
  if (is_blank(sector_addr, H743_SECTOR_SIZE))
  {
    return true;
  }

  TUF2_LOG1("Erase: sector %lu bank %lu at %08lX ... ", sector, (bank == FLASH_BANK_1) ? 1UL : 2UL, sector_addr);

  FLASH_EraseInitTypeDef erase_init = {0};
  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.Banks = bank;
  erase_init.Sector = sector;
  erase_init.NbSectors = 1;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  uint32_t sector_error = 0;
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

  if (status != HAL_OK)
  {
    TUF2_LOG1("FAILED (err=%lu)\r\n", sector_error);
    flash_error_blink();
    return false;
  }

  // Invalidate D-Cache for erased sector so subsequent reads see 0xFF
  SCB_InvalidateDCache_by_Addr((void*)sector_addr, H743_SECTOR_SIZE);
  __DSB();

  TUF2_LOG1("OK\r\n");
  return true;
}

// 32-byte aligned buffer for H7 flash word programming (must be static to reduce stack)
static uint32_t flash_word[8] __attribute__((aligned(32)));
static uint8_t local_buf[256] __attribute__((aligned(4)));

static bool flash_write_internal(uint32_t dst, const uint8_t *src, int len)
{
  // H7 requires 32-byte aligned flash word programming
  if ((dst & 0x1F) != 0)
  {
    TUF2_LOG1("Addr %08lX not aligned\r\n", dst);
    return false;
  }

  if (!flash_erase(dst))
  {
    return false;
  }

  // Copy source data to static buffer to avoid USB buffer issues during flash
  if ((uint32_t)len > sizeof(local_buf)) len = sizeof(local_buf);
  memcpy(local_buf, src, len);

  // H7 requires 256-bit (32-byte) flash word programming
  for (int i = 0; i < len; i += 32)
  {
    // Copy data from local buffer, pad with 0xFF if needed
    for (int j = 0; j < 8; j++)
    {
      int offset = i + (j * 4);
      if (offset + 4 <= len)
      {
        memcpy(&flash_word[j], local_buf + offset, 4);
      }
      else if (offset < len)
      {
        // Partial word at end
        flash_word[j] = 0xFFFFFFFF;
        memcpy(&flash_word[j], local_buf + offset, len - offset);
      }
      else
      {
        flash_word[j] = 0xFFFFFFFF;
      }
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, dst + i, (uint32_t)flash_word) != HAL_OK)
    {
      TUF2_LOG1("Flash fail at %08lX\r\n", dst + i);
      flash_error_blink();
      return false;
    }

    // Wait for flash operation to complete
    uint32_t bank = get_bank_number(dst);
    if (FLASH_WaitForLastOperation(HAL_MAX_DELAY, bank) != HAL_OK)
    {
      TUF2_LOG1("Flash wait fail\r\n");
      flash_error_blink();
      return false;
    }
  }

  // H7 cache coherency: Invalidate D-Cache for the written region before verify
  SCB_InvalidateDCache_by_Addr((void*)dst, len);
  __DSB();

  // Verify contents against local buffer
  if (memcmp((void*)dst, local_buf, len) != 0)
  {
    TUF2_LOG1("Flash verify failed\r\n");
    flash_error_blink();
    return false;
  }

  return true;
}

#endif // BOARD_PFLASH_EN

//--------------------------------------------------------------------+
// Board Memory Callouts
//--------------------------------------------------------------------+
#ifndef BOARD_PFLASH_EN
#ifdef W25Qx_SPI
uint32_t W25Qx_SPI_Transmit(uint8_t * buffer, uint16_t len, uint32_t timeout)
{
  return (uint32_t) HAL_SPI_Transmit(&_spi_flash, buffer, len, timeout);
}

uint32_t W25Qx_SPI_Receive(uint8_t * buffer, uint16_t len, uint32_t timeout)
{
  return (uint32_t) HAL_SPI_Receive(&_spi_flash, buffer, len, timeout);
}

void W25Qx_Delay(uint32_t ms)
{
  HAL_Delay(ms);
}

uint32_t W25Qx_GetTick(void)
{
  return HAL_GetTick();
}
#endif // W25Qx_SPI

//--------------------------------------------------------------------+
// Flash LL for tinyuf2
//--------------------------------------------------------------------+


extern volatile uint32_t _board_tmp_boot_addr[];
extern volatile uint32_t _board_tmp_boot_magic[];

#define TMP_BOOT_ADDR   _board_tmp_boot_addr[0]
#define TMP_BOOT_MAGIC  _board_tmp_boot_magic[0]

static void qspi_Init(void) {
  #ifdef W25Qx_QSPI
  w25qxx_Init();
  #endif
  #ifdef IS25LP064A
  CSP_QSPI_DisableMemoryMappedMode();
  CSP_QSPI_ExitQPIMODE();
  if (CSP_QUADSPI_Init() != qspi_OK) {
    TUF2_LOG1("Error initializing QSPI Flash\r\n");
  }
  #endif

}

static void qspi_EnterQPI(void) {
  #ifdef W25Qx_QSPI
  w25qxx_EnterQPI();
  #endif
}

static void qspi_Startup(void) {
  #ifdef W25Qx_QSPI
  w25qxx_Startup(qspi_DTRMode);
  #endif
  #ifdef IS25LP064A
  if (CSP_QSPI_EnableMemoryMappedMode() != qspi_OK) {
    TUF2_LOG1("Error enabling memory map for QSPI Flash\r\n");
  }
  #endif
}

static uint8_t qspi_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size) {
  #ifdef W25Qx_QSPI
  return w25qxx_Read(pData,ReadAddr,Size);
  #endif
  #ifdef IS25LP064A
  return CSP_QSPI_Read(pData, ReadAddr, Size);
  #endif
  return qspi_OK;
}

static uint8_t qspi_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
  #ifdef W25Qx_QSPI
  return w25qxx_Write(pData,WriteAddr,Size);
  #endif
  #ifdef IS25LP064A
  return CSP_QSPI_Write(pData,WriteAddr,Size);
  #endif
  return qspi_OK;
}


static void qspi_EraseChip(void) {
  #ifdef W25Qx_QSPI
  w25qxx_EraseChip();
  #endif
  #ifdef IS25LP064A
  CSP_QSPI_Erase_Chip();
  #endif
}
#endif // !BOARD_PFLASH_EN (QSPI functions)

//--------------------------------------------------------------------+
// Board Flash API
//--------------------------------------------------------------------+

#ifdef BOARD_PFLASH_EN
//--------------------------------------------------------------------+
// Internal Flash Board API (H743)
//--------------------------------------------------------------------+

void board_flash_early_init(void)
{
  // No early init needed for internal flash
}

void board_flash_init(void)
{
  // No init needed for internal flash
}

void board_flash_deinit(void)
{
  // No deinit needed for internal flash
}

uint32_t board_flash_size(void)
{
  return BOARD_FLASH_SIZE;
}

void board_flash_flush(void)
{
  // No flush needed
}

void board_flash_read(uint32_t addr, void *data, uint32_t len)
{
  // H7 cache coherency: Invalidate D-Cache before reading to ensure
  // we get actual flash contents, not stale cached data
  SCB_InvalidateDCache_by_Addr((void*)addr, len);
  memcpy(data, (void*)addr, len);
}

bool board_flash_write(uint32_t addr, void const *data, uint32_t len)
{
  // Clear any pending flash error flags before programming (per ST IAP example)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
  HAL_FLASH_Unlock();
  bool result = flash_write_internal(addr, data, len);
  HAL_FLASH_Lock();
  return result;
}

void board_flash_erase_app(void)
{
  HAL_FLASH_Unlock();

  // Erase all sectors except bootloader (sector 0)
  // Bank 1: sectors 1-7, Bank 2: sectors 0-7
  for (uint32_t sector = BOOTLOADER_SECTOR_COUNT; sector < H743_BANK1_SECTORS; sector++)
  {
    TUF2_LOG1("Erase Bank1 sector %lu\r\n", sector);

    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0;
    HAL_FLASHEx_Erase(&erase_init, &sector_error);

    // Invalidate D-Cache for erased sector so subsequent reads see 0xFF
    uint32_t sector_addr = FLASH_BASE_ADDR + (sector * H743_SECTOR_SIZE);
    SCB_InvalidateDCache_by_Addr((void*)sector_addr, H743_SECTOR_SIZE);
  }

  // Erase Bank 2 (all sectors)
  for (uint32_t sector = 0; sector < H743_BANK2_SECTORS; sector++)
  {
    TUF2_LOG1("Erase Bank2 sector %lu\r\n", sector);

    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_2;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0;
    HAL_FLASHEx_Erase(&erase_init, &sector_error);

    // Invalidate D-Cache for erased sector so subsequent reads see 0xFF
    uint32_t sector_addr = FLASH_BASE_ADDR + (H743_BANK1_SECTORS * H743_SECTOR_SIZE) + (sector * H743_SECTOR_SIZE);
    SCB_InvalidateDCache_by_Addr((void*)sector_addr, H743_SECTOR_SIZE);
  }

  __DSB();  // Ensure all cache invalidations complete
  HAL_FLASH_Lock();
}

bool board_flash_protect_bootloader(bool protect)
{
  (void)protect;
  // TODO: Implement H743 option byte write protection
  return true;
}

// For internal flash boards, these are simple stubs since we don't need
// runtime boot address selection (app always at BOARD_FLASH_APP_START)
uint32_t board_get_app_start_address(void)
{
  return BOARD_FLASH_APP_START;
}

void board_save_app_start_address(uint32_t addr)
{
  (void)addr;
  // Not used for internal flash - app always at fixed address
}

void board_clear_temp_boot_addr(void)
{
  // Not used for internal flash
}

#else // !BOARD_PFLASH_EN - QSPI flash boards (H750)
//--------------------------------------------------------------------+
// QSPI Flash Board API (H750)
//--------------------------------------------------------------------+

extern volatile uint32_t _board_tmp_boot_addr[];
extern volatile uint32_t _board_tmp_boot_magic[];

#define TMP_BOOT_ADDR   _board_tmp_boot_addr[0]
#define TMP_BOOT_MAGIC  _board_tmp_boot_magic[0]

uint32_t board_get_app_start_address(void)
{
  if (TMP_BOOT_MAGIC == 0xDEADBEEFU)
  {
    return TMP_BOOT_ADDR;
  }
  else
  {
    return BOARD_QSPI_APP_ADDR;
  }
}

void board_save_app_start_address(uint32_t addr)
{
  TMP_BOOT_MAGIC = 0xDEADBEEFU;
  TMP_BOOT_ADDR = addr;
}

void board_clear_temp_boot_addr(void)
{
  TMP_BOOT_MAGIC = 0x00U;
  TMP_BOOT_ADDR = 0x00U;
}

void board_flash_early_init(void)
{
#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
  // QSPI is initialized early to check for executable code
  qspi_flash_init(&_qspi_flash);
  // Initialize QSPI driver
  qspi_Init();
  // SPI -> QPI
  qspi_EnterQPI();
#endif // BOARD_QSPI_FLASH_EN
}

void board_flash_init(void)
{
#if defined (BOARD_SPI_FLASH_EN) && (BOARD_SPI_FLASH_EN == 1)
  // Initialize SPI peripheral
  spi_flash_init(&_spi_flash);
  // Initialize SPI drivers
  W25Qx_Init();
#endif // BOARD_SPI_FLASH_EN
}

void board_flash_deinit(void)
{
#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
  // Enable Memory Mapped Mode
  // QSPI flash will be available at 0x90000000U (readonly)
  qspi_Startup();
#endif // BOARD_QSPI_FLASH_EN
}

uint32_t board_flash_size(void)
{
  // TODO: how do we handle more than 1 target here?
  return 8*1024*1024;
}

void board_flash_flush(void)
{
  // TODO: do we need to implement this if there no caching involved?
  // maybe flush cached RAM here?
}

void board_flash_read(uint32_t addr, void * data, uint32_t len)
{
  TUF2_LOG1("Reading %lu byte(s) from 0x%08lx\r\n", len, addr);
#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
  // addr += QSPI_BASE_ADDR;
  if (IS_QSPI_ADDR(addr))
  {
    (void) qspi_Read(data, addr - QSPI_BASE_ADDR, len);
    return;
  }
#endif

#if defined (BOARD_AXISRAM_EN) && (BOARD_AXISRAM_EN == 1)
  if (IS_AXISRAM_ADDR(addr) && IS_AXISRAM_ADDR(addr + len - 1))
  {
    memcpy(data, (void *) addr, len);
    return;
  }
#endif // BOARD_AXISRAM_EN

  if (IS_PFLASH_ADDR(addr))
  {
    memcpy(data, (void *) addr, len);
    return;
  }

  {
    // Invalid address read
    __asm("bkpt #3");
  }
}

bool board_flash_write(uint32_t addr, void const * data, uint32_t len)
{
  TUF2_LOG1("Programming %lu byte(s) at 0x%08lx\r\n", len, addr);

  // For external flash
  // TODO: these should be configurable parameters
  // Page size = 256 bytes
  // Sector size = 4K bytes
#if defined (BOARD_SPI_FLASH_EN) && (BOARD_SPI_FLASH_EN == 1U)
  if (IS_SPI_ADDR(addr) && IS_SPI_ADDR(addr + len - 1))
  {
    W25Qx_Write((uint8_t *) data, (addr - SPI_BASE_ADDR), len);
    return true;
  }
#endif

#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
  if (IS_QSPI_ADDR(addr) && IS_QSPI_ADDR(addr + len - 1))
  {
    // SET_BOOT_ADDR(BOARD_AXISRAM_APP_ADDR);
    // handles erasing internally
    #ifdef IS25LP064A
    // flash needs to be erased before writing
    if (addr % IS25LP064A_SECTOR_SIZE == 0) {
      // erase 4k sector ahead of next page writes
      if (CSP_QSPI_EraseSector(addr, addr+IS25LP064A_SECTOR_SIZE) != qspi_OK) {
        TUF2_LOG1("Error erasing sector at address: %lx \r\n",addr);
      }
    }
    #endif
    if (qspi_Write((uint8_t *)data, (addr - QSPI_BASE_ADDR), len) != qspi_OK)
    {
      TUF2_LOG1("Error QSPI Flash write\r\n");
      __asm("bkpt #9");
    }
    return true;
  }
#endif

#if defined (BOARD_AXISRAM_EN) && (BOARD_AXISRAM_EN == 1)
  if (IS_AXISRAM_ADDR(addr) && IS_AXISRAM_ADDR(addr + len - 1))
  {
    // This memory is cached, DCache is cleaned in dfu_complete
    SET_BOOT_ADDR(BOARD_AXISRAM_APP_ADDR);
    memcpy((void *) addr, data, len);
    return true;
  }
#endif // BOARD_AXISRAM_EN

  // This is not a good idea for the h750 port because
  // - There is only one flash bank available
  // - There is only one sector available
  // - It will also need a config section in flash to store the boot address
  if (IS_PFLASH_ADDR(addr) && IS_PFLASH_ADDR(addr + len - 1))
  {
    // TODO: Implement this
    // SET_BOOT_ADDR(BOARD_PFLASH_APP_ADDR);
    return false;
  }

  // Invalid address write
  __asm("bkpt #4");
  return false;
}

void board_flash_erase_app(void)
{
  board_flash_init();

#if defined (BOARD_QSPI_FLASH_EN) && (BOARD_QSPI_FLASH_EN == 1)
  TUF2_LOG1("Erasing QSPI Flash\r\n");
  // Erase QSPI Flash
  (void) qspi_EraseChip();
#endif

#if defined(BOARD_SPI_FLASH_EN) && (BOARD_SPI_FLASH_EN == 1)
  TUF2_LOG1("Erasing SPI Flash\r\n");
  // Erase QSPI Flash
  (void) W25Qx_Erase_Chip();
#endif

  // TODO: Implement PFLASH erase for non-tinyuf2 sectors
  board_reset();
}

#endif // !BOARD_PFLASH_EN
