
## Deployment Scripts

Code is deployed to the robot as a Docker image. Most actions are automated with the `robot.sh` script.

### Example Uses

#### Deploying
```bash
./robot.sh -a deploy
```

This will copy and build your current workspace on the NUC. After building it will start execuiting on the NUC after stopping any previous code. A custom launch command can be given through command line arguments after `deploy`. After starting, your shell will show stdout/stderr from the NUC. Ctrl+Cing in your shell will not stop the process on the NUC.

#### Starting/Stopping
```bash
./robot.sh -a start
./robot.sh -a stop
```

These start/stop commands will start and stop the last image that has been deployed to the NUC. A custom launch command can be given through command line arguments after `start`. After starting, your shell will show stdout/stderr from the NUC. Ctrl+Cing in your shell will not stop the process on the NUC.

#### Viewing the console
```bash
./robot.sh -a watch
```

"Watching" the console will watch the output of stdout/stderr, while also showing output from before you connected. Ctrl+Cing in your shell will not stop the process on the NUC.

### Configuration
Deployment script configuration is set in `robot.env`. `ROBOT_IP` and `DEFAULT_LAUNCH` set the target system and default command to execute when starting an image. Other parameters like image/container name can be tweaked, but please don't change them on the competition robot without good reason.

### Help, I can't connect

Make sure you

- Connected to the robot_onboard WiFi network.
- Have Docker installed on your machine (a local daemon does not need to be running)
- Have your SSH key trusted by the NUC (see instructions in the computer-configuration folder)
- Recieve a response with `ping 192.168.1.22`
  - If you get a request timeout after the NUC has been powered on for a minute, power cycle the NUC

### I want to deploy the stack locally (magellan_sim)
Simply add the `-l` flag to your `robot.sh` command. Use the local flag with all other robot.sh functions as well.
```bash
./robot.sh -a deploy -l
```
### Interact with stack
To interact with the robot (rostopic, ./robot.sh -a ssh, rosnode, etc) run
```bash
source rostarget.sh
```
To interact with the sim, simply add the `--local` flag.
```bash
source rostarget.sh --local
```
