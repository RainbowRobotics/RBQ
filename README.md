
# DEV PC

### FRESH INSTALL Ubuntu 22.04 on the DEV PC 

### RUN Simulator (mujoco), motion, vision and GUI applications on the DEV PC
```sh
bash scripts/sim.bash --mujoco
```

### [OPTIONAL] Setup development environment with 3rdparty libraries and applications on the DEV PC
```sh
bash scripts/setup.bash
```

### [OPTIONAL] RUN Simulator (choreonoid), motion applications and GUI on the DEV PC
```sh
bash scripts/sim.bash --choreonoid
```

## DEPLOY

### Connect to the Robot WiFi-network

### [OPTIONAL] Deploy applications (binaries) to the Robot PC
```sh
bash scripts/deploy.bash --choreonoid
```

