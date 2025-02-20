/***************************************************************
* 
* This code shows how to use the built-in SHA256 accelerator
* on the ESP32 by calculating the double hash of a Bitcoin block
* header.
*
* See: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
*
* Other ESP32 chips, including the C3, S3, etc. may have different
* registers, but the process is very similar.
*
* Compiling:
*
* Set Arduino and events to run con CORE 0. This will let the
* hasher run unhindered at a higher priority on CORE 1.
* 
****************************************************************/
#include <Arduino.h>
#include "soc/hwcrypto_reg.h"
#include "soc/dport_reg.h"
#include "esp32/rom/ets_sys.h"
#include "freertos/task.h"
#include <esp_task_wdt.h>

#define BYTESWAP32(z) ((uint32_t)((z&0xFF)<<24|((z>>8)&0xFF)<<16|((z>>16)&0xFF)<<8|((z>>24)&0xFF)))


// Define what the Bitcoin block header looks like
typedef struct {
  uint32_t version;
  uint8_t prevHash[32];
  uint8_t merkleRoot[32];
  uint32_t ts;
  uint32_t difficulty;
  uint32_t nonce;
} HashBlock;


// Set up some variables
TaskHandle_t thandle;
volatile uint64_t totalHashes;
volatile bool keepHashing = true;


// This task will be pinned to CORE 1 (See compilation note)
void hashTask(void *taskId) {

  // The ESP32-WROOM32 uses the same registers for text and hashing
  volatile uint32_t *shaData = (uint32_t*) SHA_TEXT_BASE;
  volatile uint32_t *hash = (uint32_t*) SHA_TEXT_BASE;

  // Set up a hash block header
  HashBlock hb;

  uint32_t* hb32 = (uint32_t*) &hb;

  Serial.printf("Hashing task has started.\n");

  while(1) {

    // Set up the hash block before the tight loop
    // populating fields and doing byte swaps.

    while(keepHashing) {

      // Unrolled loop to copy the first 64 bytes of our hash block
      // into SHA256 data registers.
      shaData[0] = hb32[0];      
      shaData[1] = hb32[1];
      shaData[2] = hb32[2];      
      shaData[3] = hb32[3];      
      shaData[4] = hb32[4];      
      shaData[5] = hb32[5];      
      shaData[6] = hb32[6];      
      shaData[7] = hb32[7];      
      shaData[8] = hb32[8];      
      shaData[9] = hb32[9];      
      shaData[10] = hb32[10];      
      shaData[11] = hb32[11];      
      shaData[12] = hb32[12];      
      shaData[13] = hb32[13];      
      shaData[14] = hb32[14];      
      shaData[15] = hb32[15]; 
      
      // Tell the SHA256 hardware to start a new hash
      // and wait for it to complete.
      DPORT_REG_WRITE(SHA_256_START_REG, 1);
      while(DPORT_REG_READ(SHA_256_BUSY_REG) != 0);

      // Now copy the remaining bytes from the hash block
      // into the SHA data registers to prepare for second
      // half of hashing cycle.
      shaData[0] = hb32[16];
      shaData[1] = hb32[17];
      shaData[2] = BYTESWAP32(hb32[18]); // Timestamp
      shaData[3] = BYTESWAP32(hb32[19]); // Nonce
      shaData[4] = 0x80000000;           // Trailing bit
      shaData[5] = 0;
      shaData[6] = 0;
      shaData[7] = 0;
      shaData[8] = 0;
      shaData[9] = 0;
      shaData[10] = 0;
      shaData[11] = 0;
      shaData[12] = 0;
      shaData[13] = 0;
      shaData[14] = 0;
      shaData[15] = 0x00000280; // Total size of hash block in bits

      // Tell the SHA256 hardware that we're continuing a hash
      // and wait for it to do its thing.
      DPORT_REG_WRITE(SHA_256_CONTINUE_REG, 1);
      while(DPORT_REG_READ(SHA_256_BUSY_REG) != 0);

      // Now tell the hardware to complete the hash
      // and load it into the registers.
      DPORT_REG_WRITE(SHA_256_LOAD_REG, 1); \
      while(DPORT_REG_READ(SHA_256_BUSY_REG) != 0);

      // In the case of the ESP32, the input data
      // and output hash share the same registeers, so
      // the first hash is already right where we want it to be,
      // in the the input data area in slots 0 through 7.

      // Set up the rest of the SHA256 data area.
      shaData[8] = 0x80000000;  // Trailing bit
      shaData[9] = 0;
      shaData[10] = 0;
      shaData[11] = 0;
      shaData[12] = 0;
      shaData[13] = 0;
      shaData[14] = 0;
      shaData[15] = 0x00000100; // Size of data in bits

      // Tell the SHA256 hardware to start a new hash
      // and wait for it to complete.
      DPORT_REG_WRITE(SHA_256_START_REG, 1);
      while(DPORT_REG_READ(SHA_256_BUSY_REG) != 0);

      // Now tell the hardware to complete the hash
      // and load it into the registers.
      DPORT_REG_WRITE(SHA_256_LOAD_REG, 1); \
      while(DPORT_REG_READ(SHA_256_BUSY_REG) != 0);

      // We now have our double hash in the hardware SHA registers
      // and we can check to see if it meets the required
      // difficulty, submit it to the pool, etc.      

      
      // Increment the nonce for the next hash
      hb.nonce++;

      // Track our total hashes for calculation
      totalHashes++;
    }
  }

}


void setup() {
  
  Serial.begin(115200);

  // Enable built-in SHA256 hasher
  DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
  DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);

  // Create a hashing task pinned to core 1 
  xTaskCreatePinnedToCore(hashTask, "Hasher", 4000, (void*) 0, 2, &thandle, 1); 
}


void loop() {
  
  static uint64_t lastHashes = 0;

  // Get the number of hashes since our last check
  uint64_t hashes = totalHashes - lastHashes;

  // Get hashrate in kHashes
  double hashRate = (double) hashes / 1000.0;
  
  // Keep track of last hashes
  lastHashes = totalHashes;

  // Put the hash rate on the console.
  Serial.printf("Hash rate: %.2f kH/s\n", hashRate);

  // Calculate and show hash rate every second
  delay(1000);

}
