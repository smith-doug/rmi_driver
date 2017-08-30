# rmi_driver

This is an attempt to create a more generic and reusable driver for the robot_movement_interface.  The communication between ROS and the robot is similar to the iiwa_driver's telnet interface, but can be modified and extended with plugins.

@todo this is getting really long.  How many details should be included vs. just saying "read the doxygen"?  Some standard moves should also be created.  It will defeat the purpose of a robot-independent interface if something as simple as executing a PTP is totally different for every robot.  Fancy commands that take advantage of a specific robot's features for move types are great, but there should be some bare minimum baseline.

## Key features
- Robot specific implementation is done via plugins.  New commands and functionality that take full advantage of robot specific features can be added easily while the ROS message format remains consistent.
- Compatible with the standard ROS-I joint_trajectory_action. 
- Multi-kinematic support.
- Asynchronous sockets enable canceling of commands.
- Trajectory appending is possible.
- Socket behavior is similar to the industrial_robot_client.  
    - 1 socket is used for status messages that don't block.
    - 1 socket is used for commands that may block and should return a message when they are done.

### Behavior 
The rmi_driver package handles common functionality like handling sockets, receiving/publishing ros topics, loading plugins, connecting to robots, etc.  It uses plugins to determine what actually needs to be sent to the robot and what commands to respond to. 

It consists of:
- Driver
- Connector
- Commands and handlers  
    + RobotCommand
    + CommandHandler
    + CommandRegister



**Driver:**  
This class loads plugins, creates connections, and publishes aggregate topics.

- @todo loading parameters from a config file.  Each robot has a connection entry that contains:
    + namespace
    + ip/base port    
    + package name of the plugin
    + plugin class lookup name
    + List of joints.
- Loading the plugins.
- Creating the Connections, assigning them a plugin, and telling them to connect to their robot.
- Aggregating the joint_state messages in each connection and publishing them in 1 combined message on /joint_states.

**Connection:**  
This class handles receiving a command_list, figuring out what to do with each command, and sending it to the robot.  It uses the CommandRegister that was loaded from a plugin to determine what to do.  It will publish a command_result.  A namespace is used for each Connection to allow multiple robots.

2 sockets are used for communication with the robot.  They are (currently) referred to as Cmd (default  port 30000) and Get (Cmd + 1).  Cmd is used for commands that may require some time to process (moves, waiting for inputs).  Get is for commands that should return immediately (get version, get joint position, abort, etc).  RobotCommands will automatically select which socket to use based on their type.  

**Note: RobotCommands of type Cmd should be used for all normal commands, even if the command is meant to get some data.** @todo Think about this.  This doesn't really make sense with the way things are right now.  I need to either add a 3rd command type HighPriority or have the plugin set up all cyclical data and retrieve all desired process data cyclically.  That doesn't make sense either...  Maybe this just can't be used for checking IOs via a command outside of the flow of the command_list program?  Considering how that would require making a new /command_list_io or something.  I was trying to keep the command plugin simple, but it may be necessary to allow it to add cyclical command and publish topics on its own, like digital io state.

Each command, regardless of type, must receive a response before it can execute the next command.  

Example Get:  (note, the actual messages sent are defined by the robot specific plugin)  
```
ros->robot: "get the current joint position"  
{no delay}
ros<-robot: "1.0 2.0 3.0 4.0 5.0 6.0"  
```

Example Cmd:
```
ros->robot: "do a ptp move to 1 2 3 4 5 6"
{Delay until the robot is able to process this command, which could be immediately.  This could involve locking it into the motion buffers, adding it to an array, etc.}
ros<-robot: "done"
```
Asynchronous sockets are used so that commands can be aborted, even while waiting for a response.  If a RobotCommand of type Get is received as a command in the command_list topic, it will be regarded as a high priority message.  


**Commands**  
Plugins are used to create RobotCommands based on the contents of a Command message. Command handling consists of 3 classes:  
- A *RobotCommand* contains the data that is going to be sent to the robot.  
- A *CommandHandler* takes a robot_movement_interface::Command and creates an appropriate RobotCommand.
- A *CommandRegister* contains a list of CommandHandlers.  It selects the correct CommandHandler based on the received robot_movement_interface::Command.

Plugins should extend a CommandHandler for each command type and add them to an extended CommandRegister.  The plugin loader will load the CommandRegister and use this in the Connector.

RobotCommands consist of a vector of pairs of strings (command, values).  The base implementation's toString will create command strings in the form:
```c++
<command>[ : <values>];[<additional command or setting>[: <values>];]

Examples:
RobotCommand cmd;
cmd.setCommand("ptp joints", "1 2 3 4 5 6");  
//toString creates: ptp joints : 1 2 3 4 5 6;

cmd.addParam("speed %", "100");
cmd.addParam("overlap", "100");
//toString creates: ptp joints : 1 2 3 4 5 6; speed % : 100; overlap : 100;
```


CommandHandlers are used to create RobotCommands based on the contents of a robot_movement_interface::Command message.  Command handlers contain a sample robot_movement_interface::Command message that will be used to check if they are the correct handler for the message by the CommandRegister.  If they are, processMsg will be called and the handler will return a new RobotCommand.

Example CommandHandler that will respond to a "HELLO" command and send "hello" to the robot:
```c++
class CommandHello : public CommandHandler
{
public:
  CommandHello()
  {
    sample_msg_.command_type = "HELLO";
  }
  RobotCommandPtr processMsg(const robot_movement_interface::Command &cmd_msg) const override {
    RobotCommandPtr cmd_ptr = std::make_shared<RobotCommand>(RobotCommand::RobotCommand::CommandType::Cmd);
    cmd_ptr->setCommand("hello", "");
    return cmd_ptr;
  }
};
```

A CommandRegister contains a list of CommandHandlers.  When a CommandList message arrives, the Connector will search through the CommandHandlers by comparing each Command message with the handler's sample message criteria.  


