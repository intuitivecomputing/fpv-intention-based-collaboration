# fpv-intention-based-collaboration
This repository contains code for human-robot collaboration tasks based on first-person (user) intention.

This repo saves the video as seperated images. The folder of the images: `home/{your user's name}/collab-data`

Start recording:
```
rostopic pub /stop std_msgs/Bool "data: false" 
```


Stop recording:
```
rostopic pub /stop std_msgs/Bool "data: true" 
```
