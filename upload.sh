../esptool/esptool.py --port /dev/ttyUSB0 --baud 230400 write_flash -ff 20m -fm qio -fs 32m 0x00000 ../bin/eagle.flash.bin 0x40000 ../bin/eagle.irom0text.bin
if [ $? -eq 0 ]         # Test exit status of "make" command.
then
  (gtkterm &) 2> /dev/null
fi
