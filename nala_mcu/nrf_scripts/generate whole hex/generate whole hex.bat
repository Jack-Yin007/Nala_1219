DEL bootloader_setting.hex
DEL bootloader.hex
DEL whole.hex
pause

nrfutil settings generate --family NRF52840 --application nrf52840_xxaa.hex --application-version 0 --bootloader-version 0 --bl-settings-version 2 bootloader_setting.hex

pause

mergehex --merge nrf52840_xxaa_mbr.hex bootloader_setting.hex --output bootloader.hex
mergehex --merge bootloader.hex nrf52840_xxaa.hex s140_nrf52_7.0.1_softdevice.hex --output whole.hex

