# CoSiMA's Kinematics and Dynamics Components

Below there is a list of currently usable OROCOS RTT components.

## cosima::RTTKinDynMultiArm

This component is designed to support multiple (virtual) robots with varying degrees-of-freedom and fixed task space dimensions (Cartesian: 6). For each robot, a particular solver for kinematics and dynamics can be chosen:
* KDL
* ...

When multiple robots are loaded, the output quantities **are stacked** and send through the same OutputPorts:

| | |
|:---|:---:|
| joint-space intertia |![\begin{pmatrix} \mathbf{M}_1 &  & \mathbf{0}\\   & \ddots  & \\  \mathbf{0} &  & \mathbf{M}_r \end{pmatrix}](https://render.githubusercontent.com/render/math?math=%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7BM%7D_1%20%26%20%20%26%20%5Cmathbf%7B0%7D%5C%5C%20%20%20%26%20%5Cddots%20%20%26%20%5C%5C%20%20%5Cmathbf%7B0%7D%20%26%20%20%26%20%5Cmathbf%7BM%7D_r%20%5Cend%7Bpmatrix%7D)|
| joint-space gravity | ![\begin{pmatrix} \mathbf{g}_1\\  \vdots \\  \mathbf{g}_r \end{pmatrix}](https://render.githubusercontent.com/render/math?math=%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bg%7D_1%5C%5C%20%20%5Cvdots%20%5C%5C%20%20%5Cmathbf%7Bg%7D_r%20%5Cend%7Bpmatrix%7D)|
| joint-space coriolis | ![\begin{pmatrix} \mathbf{c}_1\\  \vdots \\  \mathbf{c}_r \end{pmatrix}](https://render.githubusercontent.com/render/math?math=%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bc%7D_1%5C%5C%20%20%5Cvdots%20%5C%5C%20%20%5Cmathbf%7Bc%7D_r%20%5Cend%7Bpmatrix%7D)|

An exemplary usage of this component looks like this:
```python
# 1) Load the lib into OROCOS RTT
import("cosima-controller")
# 2) Instantiate the component
loadComponent("KinDyn", "cosima::RTTKinDynMultiArm")
# 3) Set the Activity for execution (here at 1ms)
setActivity("KinDyn",0.001,10,ORO_SCHED_OTHER)
# 4) Add a robot incl. kinematic chain to be solved with KDL
KinDyn.addChain("KDL","kuka-iiwa-7/model.urdf","iiwa7_link_0","iiwa7_link_ee")
# 5) Configure the component to create the ports
KinDyn.configure()
# 6) Connect the InputPort to retrieve the robot feedback
var ConnPolicy cp;
connect("robot.out_kuka1_jointstate_fdb","KinDyn.in_robotstatus_port",cp)
# 7) Run the component
KinDyn.start()
```

_**Note**_: all robots (kinematic chains) need to be added **before** `configure()` can be called.