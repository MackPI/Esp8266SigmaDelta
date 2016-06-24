make clean
make
if [ $? -eq 0 ]         # Test exit status of "make" command.
then
  pkill -SIGTERM gtkterm
fi