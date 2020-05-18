# CoSiMA's Kinematics and Dynamics Components

Below there is a list of currently usable OROCOS RTT components.

## cosima::RTTKinDynMultiArm

This component is designed to support multiple (virtual) robots with varying degrees-of-freedom and fixed task space dimensions (Cartesian: 6). For each robot, a particular solver for kinematics and dynamics can be chosen:
* KDL
* ...

When multiple robots are loaded, the output quantities **are stacked** and send through the same OutputPorts:

| Data | Structure | Port |
|:---|:---:|:---|
| joint-space robot state | Stacked per individual elements|`in_robotstatus_port`|
| joint-space intertia |![\begin{matrix} \begin{pmatrix} \mathbf{M}_1 & & \mathbf{0}\\ & \ddots & \\ \mathbf{0} & & \mathbf{M}_r \end{pmatrix} & \begin{matrix} \mathbb{R}^{DoF_1 \times DoF_1}\\ \vdots \\ \mathbb{R}^{DoF_r \times DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7BM%7D_1%20%26%20%26%20%5Cmathbf%7B0%7D%5C%5C%20%26%20%5Cddots%20%26%20%5C%5C%20%5Cmathbf%7B0%7D%20%26%20%26%20%5Cmathbf%7BM%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7BDoF_1%20%5Ctimes%20DoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7BDoF_r%20%5Ctimes%20DoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)| `out_inertia_port` |
| intertia inverted |![\begin{matrix} \begin{pmatrix} \mathbf{M}^{-1}_1 & & \mathbf{0}\\ & \ddots & \\ \mathbf{0} & & \mathbf{M}^{-1}_r \end{pmatrix} & \begin{matrix} \mathbb{R}^{DoF_1 \times DoF_1}\\ \vdots \\ \mathbb{R}^{DoF_r \times DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7BM%7D%5E%7B-1%7D_1%20%26%20%26%20%5Cmathbf%7B0%7D%5C%5C%20%26%20%5Cddots%20%26%20%5C%5C%20%5Cmathbf%7B0%7D%20%26%20%26%20%5Cmathbf%7BM%7D%5E%7B-1%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7BDoF_1%20%5Ctimes%20DoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7BDoF_r%20%5Ctimes%20DoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)| `out_inertiaInv_port` |
| joint-space gravity | ![\begin{matrix} \begin{pmatrix} \mathbf{g}_1\\  \vdots \\  \mathbf{g}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{DoF_1}\\  \vdots \\  \mathbb{R}^{DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bg%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7Bg%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7BDoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7BDoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_gravity_port`|
| joint-space coriolis | ![\begin{matrix} \begin{pmatrix} \mathbf{c}_1\\  \vdots \\  \mathbf{c}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{DoF_1}\\  \vdots \\  \mathbb{R}^{DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bc%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7Bc%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7BDoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7BDoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_coriolis_port`|
| coriolis + gravity | ![\begin{matrix} \begin{pmatrix} \mathbf{c}_1 + \mathbf{g}_1\\  \vdots \\  \mathbf{c}_r + \mathbf{g}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{DoF_1}\\  \vdots \\  \mathbb{R}^{DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bc%7D_1%20&plus;%20%5Cmathbf%7Bg%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7Bc%7D_r%20&plus;%20%5Cmathbf%7Bg%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7BDoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7BDoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_coriolisAndGravity_port`|
| jacobian | ![\begin{matrix} \begin{pmatrix} \mathbf{J}_1 & & \mathbf{0}\\ & \ddots & \\ \mathbf{0} & & \mathbf{J}_r \end{pmatrix} & \begin{matrix} \mathbb{R}^{6 \times DoF_1}\\ \vdots \\ \mathbb{R}^{6 \times DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7BJ%7D_1%20%26%20%26%20%5Cmathbf%7B0%7D%5C%5C%20%26%20%5Cddots%20%26%20%5C%5C%20%5Cmathbf%7B0%7D%20%26%20%26%20%5Cmathbf%7BJ%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7B6%20%5Ctimes%20DoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7B6%20%5Ctimes%20DoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_jacobian_port`|
| jacobian dot | ![\begin{matrix} \begin{pmatrix} \mathbf{\dot{J}}_1 & & \mathbf{0}\\ & \ddots & \\ \mathbf{0} & & \mathbf{\dot{J}}_r \end{pmatrix} & \begin{matrix} \mathbb{R}^{6 \times DoF_1}\\ \vdots \\ \mathbb{R}^{6 \times DoF_r} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7B%5Cdot%7BJ%7D%7D_1%20%26%20%26%20%5Cmathbf%7B0%7D%5C%5C%20%26%20%5Cddots%20%26%20%5C%5C%20%5Cmathbf%7B0%7D%20%26%20%26%20%5Cmathbf%7B%5Cdot%7BJ%7D%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7B6%20%5Ctimes%20DoF_1%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7B6%20%5Ctimes%20DoF_r%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_jacobianDot_port`|
| task-space pose | ![\begin{matrix} \begin{pmatrix} \mathbf{p}_1\\  \vdots \\  \mathbf{p}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{7}\\  \vdots \\  \mathbb{R}^{7} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7Bp%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7Bp%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7B7%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7B7%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_cartPos_port`|
| task-space velocity | ![\begin{matrix} \begin{pmatrix} \mathbf{\dot{p}}_1\\  \vdots \\  \mathbf{\dot{p}}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{7}\\  \vdots \\  \mathbb{R}^{7} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7B%5Cdot%7Bp%7D%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7B%5Cdot%7Bp%7D%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7B7%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7B7%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_cartVel_port`|
| task-space acceleration | ![\begin{matrix} \begin{pmatrix} \mathbf{\ddot{p}}_1\\  \vdots \\  \mathbf{\ddot{p}}_r \end{pmatrix} &  \begin{matrix} \mathbb{R}^{7}\\  \vdots \\  \mathbb{R}^{7} \end{matrix} \end{matrix}](https://latex.codecogs.com/png.latex?%5Cbegin%7Bmatrix%7D%20%5Cbegin%7Bpmatrix%7D%20%5Cmathbf%7B%5Cddot%7Bp%7D%7D_1%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbf%7B%5Cddot%7Bp%7D%7D_r%20%5Cend%7Bpmatrix%7D%20%26%20%5Cbegin%7Bmatrix%7D%20%5Cmathbb%7BR%7D%5E%7B7%7D%5C%5C%20%5Cvdots%20%5C%5C%20%5Cmathbb%7BR%7D%5E%7B7%7D%20%5Cend%7Bmatrix%7D%20%5Cend%7Bmatrix%7D)|`out_cartAcc_port`|

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