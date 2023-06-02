# Rossealfs

This is a ROS 2 node for logging. It is designed to be used together with SealFS (https://gitlab.etsit.urjc.es/esoriano/sealfs). Note that it can be used without configuring SealFS (in this case, the log files will not be authenticated and forward integrity is not granted).

## Parameters 

Rossealfs accepts a YAML parameter configuration file.


 For example:

```
$> ros2 run rossealfs rossealfs --ros-args --params-file config/params.yaml
```

This file must provide the following parameters:

- **mountpoint**: String with the path of the mount point for SealFS.

- **categories**: String array with the different categories for logging. If the category's name is `thename`, the corresponding ROS2 topic iwill be  `/rossealfs/thename`. All messages published in this topic will be logged in this category.

- **files**: String array with the log file path for each category. If the mount point is `/var/sealfslog` and the file is `/misc/file.log`, the final absolute path for the log file will be `/var/sealfslog/misc/file.log`. Those files are opened in *append* mode (as required by SealFS). If the log file does not exist, it will be created. 

- **types**: String array with the ROS2 type for the category's topics,for example `std_msgs/String`.

### Example

For example, this is a configuration file:

```
rossealfs:
    ros__parameters:
       mountpoint: "/var/sealfslog"
       categories: ["explainability", "rips", "velocity"]
       files: ["/explain.log", "/rips.log", "/velocity.log"]
       types: ["std_msgs/String", "std_msgs/String", "geometry_msgs/msg/Twist"]
```

The mount point is `/var/sealfslog`. Three categories are defined:

- `explainability`: a topic named `/rossealfs/explainability` will be created to receive a `std_msgs/String` messages that will be stored in `/var/sealfslog/explain.log`.

- `rips`: a topic named `/rossealfs/rips` will be created to receive `std_msgs/String` messages that will be stored in `/var/sealfslog/rips.log`.

- `velocity`: a topic named `/rossealfs/velocity` will be created to receive serialized `geometry_msgs/msg/Twist` messages that will be stored in `/var/sealfslog/explain.log`.


By default (i.e. without a parameters file) *rossealfs* defines one plain text log category named **all**: it will be subscribed to a topic of `std_msgs/String` named `/rossealfs/all`. The corresponding plain text log file will be stored in `/var/log/rossealfs-all.log`.  

## Decoders

If there is a custom Decoder class implemented for the type, it will be used to write the target file log. Currently, there are two custom decoders implemented:

- `StringDecoder.cpp`: If the type is `std_msgs/String`, the corresponding log file will be a plain text file. Each string published in the topic will be transformed to a text line (an end-of-line `\n` is appended to the string). Therefore, this type must be used for plain text logs.

- `RosoutDecoder.cpp`: If the type is `rcl_interfaces/msg/Log`, the log file will be a plain text file with all the fields of a ROSOUT message.

The  **Decoder** superclass is used to write the rest of message types. In this case, the log file will store the published messages in binary format, as serialized messages (ROS2 standard serialization). For each received message, two contigous writes are performed:
       
- an `int64_t` integer with the length of the serialized ROS2 message (little-endian).
- the rcl serialized message buffer.

To add a new custom Decoder class, create a derived class `<TypeName>Decoder.cpp` and modify `DecoderFactory.cpp`. 

