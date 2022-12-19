::You can find the softdevice code number by typing
::nrfutil pkg generate --help 

nrfutil pkg generate --hw-version 52 --application-version 1 --application nrf52840_xxaa.hex --sd-req 0xca --key-file private.key app_dfu_package.zip

pause
