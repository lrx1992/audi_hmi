ps -fe | grep "audi_lamp" | awk '{print $2}' |xargs kill -9
nohup python /home/pi/src/rpi-ws281x-python/examples/audi_lamp.py -m 2 >/dev/null &

