
::nrfjprog -e -f NRF52
nrfjprog --eraseall -f NRF52
pause

nrfjprog --program whole.hex --verify -f NRF52

pause

nrfjprog --reset -f NRF52
pause

