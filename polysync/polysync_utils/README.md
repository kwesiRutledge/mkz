## Description
Create short alias for cmd's in polysync

## Usage of polysync-utils

### 0. Before you start

Compile rnr_control and copy the binary file to the root directory.
```
cd rnr_control
make
cp ./bin/polysync-rnr-control-c ../../
```


### 1. Start the service (you don't need to do this for MKZ. It's done by the computer on the MKZ.)

As host, start polysync service by typing
``` 
sh manager_start.sh
```

Alternatively, you can specify the working mode by typing
```
sh set_hardware.sh      # For runtime
```

```
sh set_standby.sh       # For testing
```

```
sh set_replay.sh        # For replaying data
```

### 2. Recording

To start recording, type
```
sh record_start.sh session_name
```

The data collected will be saved to "~/.local/share/polysync/" by default.

To stop recording, type
```
sh record_stop.sh
```

### 3. Replaying

To replay a session you record previously (you don't need to specify the searching path), type
```
sh replay_start.sh session_name
```

To stop the replaying, type
```
sh replay_stop.sh
```

Alternatively, you can do this in `polysync-core-studio`.


### 4. Stop the service (You don't need to do this for MKZ. The person in charge of MKZ should do this using MKZ's computer. You may need this after replaying.)

```
sh manager_stop.sh
sh clean.sh
```
