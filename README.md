# Inverse Kinematics 
Simple implementation of the inverse kinematics theory applied to robots.

## Requirements 

in order to install the libs required by this tool

```shell 
pip3 install -r requirements.txt
```

### Python version
```shell 
python 3.*
```

## Configuration
The configs files (/resources/configs/*.yml) allows to dynamically create the robot structure (in the template below it is provided the example of a robot arm with 2 joints). In this tool a robot is considered as a set of arms connected by joints.

#### Config structure
``` yaml
n_dims: 3
Joints:  
    - id: 0
      actuators:     
          - type: Z_AXIS_MOTOR
            start_angle: 0
            constraints:         
              min: -.inf 
              max: .inf
    - id: 1
      actuators: 
          - type: X_AXIS_MOTOR
            start_angle: 0
            constraints:         
              min: -.inf 
              max: .inf
    - id: 2
      actuators: []
Geometry:
  vertices: [
    [0, 0, 0],
    [0, 0, 10],
    [0, 0, 20]    
  ]
  edges: [
    [1], [2], []
  ]
  leaf_joint: [2]
```

+ **Joints**: contains the list of joints.
    + **id**: numeric value (starting from index 0) that allows to map that joint into the robot geometry.
    + **actuators**: list of actuators (now available x, y, and z rotation).
        - TODO: start_angle and constraints are not yet implemented and must be filled as represented in the example.
+ **Geometry**: contains informations about the robot geometry.
    + **vertices**: list of vectors describing the initial position of each joint (the joint id allows to map each joint to its location).
    + **edges**: adjacency list representing the actual connection of the vertices (joints).

## Execution
+ use yaml templates (contained in /resources/configs/) or create your custom configuration file.
+ change variable **CONFIG_PATH** into ***main.py*** in order to point to your config file.
```python                    

CONFIG_PATH = os.path.join('.', 'resources', 'configs', 'armConfig - 2 joints.yml')

if __name__ == "__main__":    
    arm = ArmConfigParser(path=CONFIG_PATH).parse()    

```
+ run:
```shell 
python main.py
```


### Example 
![IKGIF](./imgs/gif/robotArmInvKinem.gif) 
