::SDK11 can update Softdevice, app and bootloader 
::here we only show app updating
::app_new is the new app image
nrfutil.exe dfu genpkg --application app_new.hex --application-version 1 SDK11_app_s132.zip

pause