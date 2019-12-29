
#ssh -t hadoop@10.101.35.209 "sudo touch /etc/b.txt"

#ps -fe | grep "audi_lamp" | awk '{print $2}' |xargs kill -9
#nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 2 >/dev/null &



stop() {
  ssh -t pi@192.168.1.65 "ps -fe | grep 'audi_lamp' | awk '{print $2}' |xargs kill -9"
  ssh -t pi@192.168.1.65 "nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 0 >/dev/null &"
  ssh -t pi@192.168.1.66 "ps -fe | grep 'audi_lamp' | awk '{print $2}' |xargs kill -9 ; nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 0 >/dev/null &"
}

start() {
  ssh -t pi@192.168.1.65 "ps -fe | grep "audi_lamp" | awk '{print $2}' |xargs kill -9 ; nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 2 >/dev/null &"
  ssh -t pi@192.168.1.66 "ps -fe | grep "audi_lamp" | awk '{print $2}' |xargs kill -9 ; nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 2 >/dev/null &"  
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac
