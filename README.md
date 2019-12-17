# audi_hmi
ros_audi_hmi_backend
# Install docker

sudo apt-get remove docker docker-engine docker.io

sudo apt-get install apt-transport-https ca-certificates curl gnupg2 software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=amd64] https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

sudo apt-get update
sudo apt-get install docker-ce

## Enable insecure

sudo vim /etc/docker/daemon.json

{ "insecure-registries":["10.101.35.207:5000"] }

sudo service docker restart


## Get docker

docker pull 10.101.35.207:5000/hmi_sim:1.0


------------------------------------------

# Usage

## Run container:

    sudo docker run -d -it --net=host --rm -p 11311:11311 --name hmi_sim hmi_sim /bin/bash -c /home/start_hmi_sim.sh

## View status:

    sudo docker ps -a

## Stop container:

    sudo docker stop hmi_sim

wait for a while


# Config your own ros config

    export ROS_MASTER_URI=http://10.101.35.210:11311
    export ROS_IP={your_ip_address}
