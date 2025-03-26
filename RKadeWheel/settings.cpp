#include "settings.h"

uint8_t SettingsEEPROM::calcChecksum() {
  uint8_t i, checksum;

  checksum = ~((uint8_t*)this)[0];
  for (i = 1; i < sizeof(SettingsEEPROM) - 1; i++)
    checksum = checksum ^ ~((uint8_t*)this)[i];
  return checksum;
}
