::merge softdevice, app and bootloader to generate one hex for the programming
::if you intend to merge more than 3 hex files, use mergehex.exe twice
mergehex.exe --merge s132_nrf52_2.0.0_softdevice.hex app.hex bootloader.hex --output whole.hex

nrfjprog.exe --eraseall -f NRF52
nrfjprog.exe --program whole.hex --verify -f NRF52 
::write 1 to indicate that app is valid
nrfjprog.exe --memwr 0x0007F000 --val 0x01 --verify -f NRF52
::the following two commands are used to enable PIN RESET          
::nrfjprog.exe --memwr 0x10001200 --val 0x00000015 --verify -f NRF52    
::nrfjprog.exe --memwr 0x10001204 --val 0x00000015 --verify -f NRF52   
nrfjprog.exe --reset -f NRF52

pause

