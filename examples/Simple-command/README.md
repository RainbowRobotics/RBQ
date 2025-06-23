# 🚀 Simple-command

## 🛠️ Build the Simple-command Code

* Run the following command to build the project:

    ```bash
    bash scripts/example_build.bash
    ```
    * *arguments:*
        - `--help` : Display help message and exit.
        - `--no-cache` : rebuild.
---


## 🧪 Test

* Run `simulator` script
    ```bash
    bash scripts/sim.bash
    ```

* Run `Simple-command` process
    ```bash
    sudo ./examples/bin/Simple-command
    ```
  <table>
    <tr>
      <th style="text-align:center;">✅ Button Action</th>
      <th style="text-align:center;">Sit</th>
      <th style="text-align:center;">Stance</th>
      <th style="text-align:center;">Walk</th>
      <th style="text-align:center;">Stairs</th>
      <th style="text-align:center;">Running</th>
    </tr>
    <tr>
      <th style="text-align:center;">⌨️ Keyboard key</th>
      <th style="text-align:center;">1</th>
      <th style="text-align:center;">2</th>
      <th style="text-align:center;">3</th>
      <th style="text-align:center;">4</th>
      <th style="text-align:center;">5</th>
    </tr>
  </table>

    <img src="../../resources/gifs/simple_motion.gif" width="640" alt="Simple Motion Demo">

## 🚀 Deploy and Run in the real Robot

* Connect your PC to the robot's Wi-Fi.
  
  📝 *Wi-Fi SSID: `RBQ_xxxx` (replace `xxxx` with the specific identifier of your robot's Wi-Fi).*

* Copy the binary to the robot pc
    ```bash
    scp examples/bin/Simple-command rbq@192.168.0.10:~/rbq_ws/examples/bin/.
    ```

* Secure shell to the robot pc
    ```bash
    ssh rbq@192.168.0.10
    ```

* Run the process on robot pc
    ```bash
    cd ~/rbq_ws && sudo ./examples/bin/Simple-command
    ```
  <table>
    <tr>
      <th style="text-align:center;">✅ Button Action</th>
      <th style="text-align:center;">Sit</th>
      <th style="text-align:center;">Stance</th>
      <th style="text-align:center;">Walk</th>
      <th style="text-align:center;">Stairs</th>
      <th style="text-align:center;">Running</th>
    </tr>
    <tr>
      <th style="text-align:center;">⌨️ Keyboard key</th>
      <th style="text-align:center;">1</th>
      <th style="text-align:center;">2</th>
      <th style="text-align:center;">3</th>
      <th style="text-align:center;">4</th>
      <th style="text-align:center;">5</th>
    </tr>
  </table>


---
## ⚠️ **Software Version Compatibility**

Make sure that the **robot's onboard software version** matches the **software version installed on your development PC**.

* ✅ We recommend updating both environments to the latest official release before deployment. You can download the latest release from the [official RBQ software repository](https://github.com/RainbowRobotics/RBQ/releases).
