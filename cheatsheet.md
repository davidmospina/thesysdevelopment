# Docker:


## create docker network with specific IP range:
```
docker network create --subnet=192.168.56.0/24 ursim_net	0/24 is a range from 0 to 255
```

## Launch a simulator with a robot (change ip and name for a new robot):
```
docker run --rm -it  --net ursim_net --ip 192.168.56.101 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series
```
```
docker run --rm -it  --net ursim_net --ip 192.168.56.102 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim2 universalrobots/ursim_e-series
```