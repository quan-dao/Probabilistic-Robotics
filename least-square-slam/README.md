[//]: # (Image References)
[pose_graph]: ./images/pose_graph.png
[graph_slam_components]: ./images/graph_slam_components.png
[pose_pose_constraint]: ./images/pose_pose_constraint.png
[pose_pose_dataset]: ./images/ls_slam_pose_pose.gif
[intel_dataset]: ./images/ls_slam_intel_dataset.gif
[dlr_dataset]: ./images/ls_slam_dlr_dataset.gif


# Intro
This folder is to fulfill the Least-square SLAM framework implemented in Sheet 10 of the course [Robot Mapping](http://ais.informatik.uni-freiburg.de/teaching/ws15/mapping/)

The Least-square SLAM is a graph-based SLAM algorithm whose graph comprises of robot pose at diffrenet time instance and landmarks position  (Fig. 1). A node of this graph is either a robot pose or a landmark's position. An edge is a constraint between two poses or between one pose and a landmark.    

![alt text][pose_graph]

Fig.1 An example of graph in graph-based SLAM

The formation of such a pose graph given the sequence of control signal <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BU%7D%20%3D%20%5Cleft%5C%7Bu_1%2C%20u_2%2C%20%5Cdots%20%5Cright%5C%7D" alt="\textbf{U} = \left\{u_1, u_2, \dots \right\}" /> and sequence of observation <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BZ%7D%20%3D%20%5Cleft%5C%7Bz_1%2C%20z_2%2C%20%5Cdots%20%5Cright%5C%7D" alt="\textbf{Z} = \left\{z_1, z_2, \dots \right\}" /> is carried out by the Front-end of a graph-based SLAM algorithm, while the Back-end interprets the resulted graph into a cost function and optimize this function to find the optimal nodes position (Fig.2). 

![alt text][graph_slam_components]

Fig.2 The interaction between two ends of graph-based SLAM

The implementation here just covers the Back-end while assume that the poses graph is already available.

# Graph Optimization
Before progressing further, the state vector needs to be defined so that the target of the algorithm is clear. Since graph SLAM aims to address the ***full SLAM*** which is estimatation of robot trajectory and environment map, the state vector is made of all robot poses and location of all landmarks position.  

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D%20%3D%20%5Cleft%5B%5Ctextbf%7Bx%7D_1%2C%20%5Ctextbf%7Bx%7D_2%2C%20%5Cdots%2C%20%5Ctextbf%7Bx%7D_T%2C%20m_1%2C%20m_2%2C%20%5Cdots%2C%20m_n%20%5Cright%5D" alt="\textbf{x} = \left[\textbf{x}_1, \textbf{x}_2, \dots, \textbf{x}_T, m_1, m_2, \dots, m_n \right]" />

Here <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> is robot's 2D pose in world frame at time instance <img src="https://tex.s2cms.ru/svg/t" alt="t" />.

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t%20%3D%20%5Cleft%5Bx_t%2C%20y_t%2C%20%5Ctheta_t%20%5Cright%5D%5ET" alt="\textbf{x}_t = \left[x_t, y_t, \theta_t \right]^T" />

The map in this implementation consists of point landmarks. Therefore, a landmark is described by solely its position in <img src="https://tex.s2cms.ru/svg/m_j" alt="m_j" /> is 2D location of landmark <img src="https://tex.s2cms.ru/svg/j" alt="j" /> in world frame. 

<img src="https://tex.s2cms.ru/svg/m_j%20%3D%20%5Cleft%5Bm_%7Bj%2C%20x%7D%2C%20m_%7Bj%2C%20y%7D%20%5Cright%5D%5ET" alt="m_j = \left[m_{j, x}, m_{j, y} \right]^T" /> 

## 1. Cost Function Construction
An edge connecting node <img src="https://tex.s2cms.ru/svg/i" alt="i" /> and node <img src="https://tex.s2cms.ru/svg/j" alt="j" /> stems from the measurement <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D" alt="z_{ij}" />. This measurement can be produced by the motion model, the measurement model or the loop closure procedure. Since the generation of graph edges is handled by the Front-end, at this point, an edge is regarded as the homogeneous transformation from one node to another (assume node position is expressed in homogeneous coordinate). The uncertainty of measurement <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D" alt="z_{ij}" /> is encoded in its infomation matrix <img src="https://tex.s2cms.ru/svg/%5COmega_%7Bij%7D" alt="\Omega_{ij}" />.   

Depend on the nature of 2 nodes that an edge connects, the constraint introduced by this edge is classified as a pose-pose constraint or pose-landmark constraint.

### 1.1 Pose-pose Constraint
Let <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D%20%3D%20%5Cleft%5Bt_%7Bij%7D%5ET%2C%20%5Ctheta_%7Bij%7D%20%5Cright%5D%5ET" alt="z_{ij} = \left[t_{ij}^T, \theta_{ij} \right]^T" /> and <img src="https://tex.s2cms.ru/svg/%5COmega_ij" alt="\Omega_ij" /> be the measurement and the corresponding information matrix of the edge connects node <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_i" alt="\textbf{x}_i" /> and node <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_j" alt="\textbf{x}_j" /> (Fig.3).

![alt text][pose_pose_constraint]

Fig.3 Pose-pose constraint (image in coursety of [Robot Mapping](http://ais.informatik.uni-freiburg.de/teaching/ws15/mapping/))
 
Let <img src="https://tex.s2cms.ru/svg/X_i%2C%20X_j" alt="X_i, X_j" /> be the homogeneous transformation matrices represent pose of <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_i" alt="\textbf{x}_i" /> and <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_j" alt="\textbf{x}_j" /> in the world frame, respectively. <img src="https://tex.s2cms.ru/svg/Z_%7Bij%7D" alt="Z_{ij}" /> is the transformation matrix representing <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D" alt="z_{ij}" />.

<img src="https://tex.s2cms.ru/svg/%5Cbegin%7Bmatrix%7D%0AX_i%20%3D%20%5Cbegin%7Bbmatrix%7DR_i%20%26%20t_i%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%20%3D%20%0A%5Cbegin%7Bbmatrix%7D%0A%5Ccos%5Ctheta_i%20%26%20-%5Csin%5Ctheta_i%20%26%20x_i%20%5C%5C%0A%5Csin%5Ctheta_i%20%26%20%5Ccos%5Ctheta_i%20%26%20y_i%20%5C%5C%0A0%20%26%200%20%26%201%0A%5Cend%7Bbmatrix%7D%2C%0A%26%26%26%0AZ_%7Bij%7D%20%3D%20%5Cbegin%7Bbmatrix%7DR_%7Bij%7D%20%26%20t_%7Bij%7D%20%5C%5C%200%20%26%201%5Cend%7Bbmatrix%7D%0A%5Cend%7Bmatrix%7D" alt="\begin{matrix}
X_i = \begin{bmatrix}R_i &amp; t_i \\ 0 &amp; 1 \end{bmatrix} = 
\begin{bmatrix}
\cos\theta_i &amp; -\sin\theta_i &amp; x_i \\
\sin\theta_i &amp; \cos\theta_i &amp; y_i \\
0 &amp; 0 &amp; 1
\end{bmatrix},
&amp;&amp;&amp;
Z_{ij} = \begin{bmatrix}R_{ij} &amp; t_{ij} \\ 0 &amp; 1\end{bmatrix}
\end{matrix}" />

According to Fig.3, estimation of <img src="https://tex.s2cms.ru/svg/X_j" alt="X_j" /> based on <img src="https://tex.s2cms.ru/svg/X_i" alt="X_i" /> and <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D" alt="z_{ij}" /> is 

<img src="https://tex.s2cms.ru/svg/%5Chat%7BX%7D_j%20%3D%20X_i%20%5Ccdot%20Z_%7Bij%7D" alt="\hat{X}_j = X_i \cdot Z_{ij}" />

Let <img src="https://tex.s2cms.ru/svg/E_%7Bij%7D" alt="E_{ij}" /> be the transformation from <img src="https://tex.s2cms.ru/svg/%5Chat%7BX%7D_j" alt="\hat{X}_j" /> to <img src="https://tex.s2cms.ru/svg/X_j" alt="X_j" />; hence,

<img src="https://tex.s2cms.ru/svg/%5Chat%7BX%7D_j%20%5Ccdot%20E_%7Bij%7D%20%3D%20X_j" alt="\hat{X}_j \cdot E_{ij} = X_j" />

As a result, the error introduces by this edge is

<img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%20%3D%20t2v(E_%7Bij%7D)%20%3D%20t2v(Z%5E%7B-1%7D_%7Bij%7D%20%5Ccdot%20X_i%5E%7B-1%7D%20%5Ccdot%20X_j)%20" alt="e_{ij} = t2v(E_{ij}) = t2v(Z^{-1}_{ij} \cdot X_i^{-1} \cdot X_j) " />

Here, <img src="https://tex.s2cms.ru/svg/t2v(%5Ccdot)" alt="t2v(\cdot)" /> is the function that extracts the translation vector and the angle of rotation from a 2D transformation matrix. Expand equation above to get

<img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%0AR_%7Bij%7D%5ET%5Cleft%5BR_i%5ET%5Cleft(t_j%20-%20t_i%20%5Cright)%20-%20t_%7Bij%7D%20%5Cright%5D%20%5C%5C%0A%5Ctheta_j%20-%20%5Ctheta_i%20-%20%5Ctheta_%7Bij%7D%0A%5Cend%7Bbmatrix%7D" alt="e_{ij} = \begin{bmatrix}
R_{ij}^T\left[R_i^T\left(t_j - t_i \right) - t_{ij} \right] \\
\theta_j - \theta_i - \theta_{ij}
\end{bmatrix}" />

### 1.2 Pose-landmark Constraint
The constraint of an edge connecting a pose <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_i" alt="\textbf{x}_i" /> and a landmark position <img src="https://tex.s2cms.ru/svg/m_j" alt="m_j" /> can be derived in the same manner as pose-pose constraint. Notice that the state of a landmark does not containt its orientation, this means the measurement <img src="https://tex.s2cms.ru/svg/z_%7Bij%7D" alt="z_{ij}" /> is just the translation from <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_i" alt="\textbf{x}_i" /> to <img src="https://tex.s2cms.ru/svg/m_j" alt="m_j" />. The error equation in the presvious section becomes

<img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%20%3D%20R_i%5ET%5Cleft(m_j%20-%20t_i%20%5Cright)%20-%20z_%7Bij%7D%20" alt="e_{ij} = R_i^T\left(m_j - t_i \right) - z_{ij} " />

### 1.3 Global Cost Function
With the error introduced by every edge in the graph being defined, the global cost function is the sum of all errors' <img src="https://tex.s2cms.ru/svg/%5COmega" alt="\Omega" />-norm.

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BF%7D(%5Ctextbf%7Bx%7D)%20%3D%20%0A%5Csum_%7Bij%7De_%7Bij%7D%5ET%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%20%5Ccdot%20%5COmega_%7Bij%7D%20%5Ccdot%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%0A" alt="\textbf{F}(\textbf{x}) = 
\sum_{ij}e_{ij}^T\left(\textbf{x}_i, \textbf{x}_j \right) \cdot \Omega_{ij} \cdot e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)
" />

Rewrite <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" /> as a function of the whole state vector <img src="https://tex.s2cms.ru/svg/textbf%7Bx%7D" alt="textbf{x}" />, meaning

<img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D%5Cright)%20%3D%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)" alt="e_{ij}\left(\textbf{x}\right) = e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)" />

Note that though the argument of <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" /> is changed from just two state <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_i" alt="\textbf{x}_i" /> and <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_j" alt="\textbf{x}_j" /> to the full state <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D" alt="\textbf{x}" />, the fact that <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" /> does not depend on the other state keeps the value of <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" /> exactly the same.

With new definition of <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" />, the cost function becomes

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BF%7D(%5Ctextbf%7Bx%7D)%20%3D%20%5Csum_%7Bij%7De_%7Bij%7D%5ET%5Cleft(%5Ctextbf%7Bx%7D%5Cright)%20%5Ccdot%20%5COmega_%7Bij%7D%20%5Ccdot%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D%5Cright)%20" alt="\textbf{F}(\textbf{x}) = \sum_{ij}e_{ij}^T\left(\textbf{x}\right) \cdot \Omega_{ij} \cdot e_{ij}\left(\textbf{x}\right) " />

## 2. Cost Function Linearization
Apply Taylor expandsion on <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%5Cleft(%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D%20%5Cright)" alt="e_{ij}\left(\hat{\textbf{x}} + \Delta \textbf{x} \right)" /> with <img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D" alt="\hat{\textbf{x}}" /> is an arbitrary value of <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D" alt="\textbf{x}" /> and <img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctextbf%7Bx%7D" alt="\Delta \textbf{x}" /> is a small step from this value

<img src="https://tex.s2cms.ru/svg/e_%7Bij%7D%5Cleft(%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D%20%5Cright)%20%3D%20e_%7Bij%7D%5Cleft(%5Chat%7B%5Ctextbf%7Bx%7D%7D%5Cright)%20%2B%20J_%7Bij%7D%5CDelta%20%5Ctextbf%7Bx%7D" alt="e_{ij}\left(\hat{\textbf{x}} + \Delta \textbf{x} \right) = e_{ij}\left(\hat{\textbf{x}}\right) + J_{ij}\Delta \textbf{x}" />

Here <img src="https://tex.s2cms.ru/svg/J_%7Bij%7D" alt="J_{ij}" /> is the Jacobian of <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D(%5Ctextbf%7Bx%7D)" alt="e_{ij}(\textbf{x})" />

<img src="https://tex.s2cms.ru/svg/J_%7Bij%7D(%5Ctextbf%7Bx%7D)%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D%7D" alt="J_{ij}(\textbf{x}) = \frac{\partial e_{ij}(\textbf{x})}{\partial \textbf{x}}" />

The structure of <img src="https://tex.s2cms.ru/svg/J_%7Bij%7D" alt="J_{ij}" /> is elaborarted in the following section. Substitue the Taylor expandsion of <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D" alt="e_{ij}" /> into the cost function,

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BF%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D)%20%3D%20%5Csum_%7Bij%7De_%7Bij%7D%5ET%5Cleft(%5Chat%7B%5Ctextbf%7Bx%7D%7D%5Cright)%20%5COmega_%7Bij%7D%20e_%7Bij%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%2B%20%0A2%20%5CDelta%20%5Ctextbf%7Bx%7D%5ET%20%5Csum_%7Bij%7D%20J_%7Bij%7D%5ET(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%5COmega_%7Bij%7D%20e_%7Bij%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%2B%20%0A%5CDelta%20%5Ctextbf%7Bx%7D%5ET%20%5Cleft(%5Csum_%7Bij%7D%20J_%7Bij%7D%5ET(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%5COmega_%7Bij%7D%20J_%7Bij%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%5Cright)%20%5CDelta%20%5Ctextbf%7Bx%7D%0A" alt="\textbf{F}(\hat{\textbf{x}} + \Delta \textbf{x}) = \sum_{ij}e_{ij}^T\left(\hat{\textbf{x}}\right) \Omega_{ij} e_{ij}(\hat{\textbf{x}}) + 
2 \Delta \textbf{x}^T \sum_{ij} J_{ij}^T(\hat{\textbf{x}}) \Omega_{ij} e_{ij}(\hat{\textbf{x}}) + 
\Delta \textbf{x}^T \left(\sum_{ij} J_{ij}^T(\hat{\textbf{x}}) \Omega_{ij} J_{ij}(\hat{\textbf{x}}) \right) \Delta \textbf{x}
" />

Let,

<img src="https://tex.s2cms.ru/svg/%20b%20%3D%20%5Csum_%7Bij%7D%20J_%7Bij%7D%5ET(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%5COmega_%7Bij%7D%20e_%7Bij%7D%20" alt=" b = \sum_{ij} J_{ij}^T(\hat{\textbf{x}}) \Omega_{ij} e_{ij} " />

<img src="https://tex.s2cms.ru/svg/%20H%20%3D%20%5Csum_%7Bij%7D%20J_%7Bij%7D%5ET(%5Chat%7B%5Ctextbf%7Bx%7D%7D)%20%5COmega_%7Bij%7D%20J_%7Bij%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D)" alt=" H = \sum_{ij} J_{ij}^T(\hat{\textbf{x}}) \Omega_{ij} J_{ij}(\hat{\textbf{x}})" />

A quadratic form emerge from the cost function

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BF%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D)%20%3D%20%5Ctextit%7Bconst%7D%20%2B%202%20%5CDelta%20%5Ctextbf%7Bx%7D%5ET%20b%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D%5ET%20H%20%5CDelta%20%5Ctextbf%7Bx%7D" alt="\textbf{F}(\hat{\textbf{x}} + \Delta \textbf{x}) = \textit{const} + 2 \Delta \textbf{x}^T b + \Delta \textbf{x}^T H \Delta \textbf{x}" />

For <img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D" alt="\hat{\textbf{x}} + \Delta \textbf{x}" /> to be a local minimum of <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BF%7D" alt="\textbf{F}" />, <img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctextbf%7Bx%7D" alt="\Delta \textbf{x}" /> needs to satisfy the equation

<img src="https://tex.s2cms.ru/svg/%5Cfrac%7B%5Cpartial%20%5Ctextbf%7BF%7D(%5Chat%7B%5Ctextbf%7Bx%7D%7D%20%2B%20%5CDelta%20%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20%5CDelta%20%5Ctextbf%7Bx%7D%7D%20%3D%200" alt="\frac{\partial \textbf{F}(\hat{\textbf{x}} + \Delta \textbf{x})}{\partial \Delta \textbf{x}} = 0" />

The quation above results in the optimal step from <img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D" alt="\hat{\textbf{x}}" />

<img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctextbf%7Bx%7D%20%3D%20-H%5E%7B-1%7D%20b" alt="\Delta \textbf{x} = -H^{-1} b" />

To find <img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctextbf%7Bx%7D" alt="\Delta \textbf{x}" /> and solve the graph optimization, the contribution of each edge to <img src="https://tex.s2cms.ru/svg/b" alt="b" /> and <img src="https://tex.s2cms.ru/svg/H" alt="H" /> need to be investigated.

### 2.1 Structure of Jacobian
Considering an arbitrary edge connecting node <img src="https://tex.s2cms.ru/svg/i" alt="i" /> to node <img src="https://tex.s2cms.ru/svg/j" alt="j" />, the Jacobian of the error introduces by this edge <img src="https://tex.s2cms.ru/svg/e_%7Bij%7D(%5Ctextbf%7Bx%7D)" alt="e_{ij}(\textbf{x})" /> is

<img src="https://tex.s2cms.ru/svg/J_%7Bij%7D%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Ctextbf%7Bx%7D%7D%20%3D%20%5Cleft(0%20%0A%5Cdots%20%0A%5Cfrac%7B%5Cpartial%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_i%7D%20%0A%5Cdots%20%0A%5Cfrac%7B%5Cpartial%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_j%7D%20%0A%5Cdots%20%0A0%20%5Cright)" alt="J_{ij} = \frac{\partial e_{ij}(\textbf{x})}{\textbf{x}} = \left(0 
\dots 
\frac{\partial e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)}{\partial \textbf{x}_i} 
\dots 
\frac{\partial e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)}{\partial \textbf{x}_j} 
\dots 
0 \right)" />

Let <img src="https://tex.s2cms.ru/svg/A_%7Bij%7D%2C%20B_%7Bij%7D" alt="A_{ij}, B_{ij}" /> respectively denote <img src="https://tex.s2cms.ru/svg/%5Cfrac%7B%5Cpartial%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_i%7D" alt="\frac{\partial e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)}{\partial \textbf{x}_i}" /> and <img src="https://tex.s2cms.ru/svg/%5Cfrac%7B%5Cpartial%20e_%7Bij%7D%5Cleft(%5Ctextbf%7Bx%7D_i%2C%20%5Ctextbf%7Bx%7D_j%20%5Cright)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_j%7D" alt="\frac{\partial e_{ij}\left(\textbf{x}_i, \textbf{x}_j \right)}{\partial \textbf{x}_j}" />, <img src="https://tex.s2cms.ru/svg/J_%7Bij%7D" alt="J_{ij}" /> becomes

<img src="https://tex.s2cms.ru/svg/J_%7Bij%7D%20%3D%20%5Cleft(0%20%0A%5Cdots%20%0AA_%7Bij%7D%20%0A%5Cdots%20%0AB_%7Bij%7D%0A%5Cdots%20%0A0%20%5Cright)" alt="J_{ij} = \left(0 
\dots 
A_{ij} 
\dots 
B_{ij}
\dots 
0 \right)" />

As can be seen from this equation, <img src="https://tex.s2cms.ru/svg/J_%7Bk%7D" alt="J_{k}" /> is sparse.

### 2.2 Consequence of Sparseness
Substitute <img src="https://tex.s2cms.ru/svg/J_%7Bij%7D" alt="J_{ij}" /> established above into the formula of <img src="https://tex.s2cms.ru/svg/H_%7Bij%7D" alt="H_{ij}" /> (an element in the sum resulting in <img src="https://tex.s2cms.ru/svg/H" alt="H" />) and <img src="https://tex.s2cms.ru/svg/b_%7Bij%7D" alt="b_{ij}" />

<img src="https://tex.s2cms.ru/svg/%0AH_%7Bij%7D%20%3D%20J_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20J_%7Bij%7D%20%3D%20%0A%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%20%5Cdots%20%5C%5C%20A%5ET_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%20B%5ET_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%200%20%5Cend%7Bbmatrix%7D%20%0A%5COmega_%7Bij%7D%20%5Cbegin%7Bbmatrix%7D%200%20%26%20%5Cdots%20%26%20A_%7Bij%7D%20%26%20%5Cdots%20%26%20B_%7Bij%7D%20%26%20%5Cdots%20%26%200%20%5Cend%7Bbmatrix%7D%20%5C%5C%3D%5Cbegin%7Bbmatrix%7D%20%0A0%20%26%20%5Cdots%20%26%200%20%26%20%5Cdots%20%26%200%20%26%20%5Cdots%20%26%200%20%5C%5C%0A%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%5C%5C%0A0%20%26%20%5Cdots%20%26%20A%5ET_%7Bij%7D%5COmega_%7Bij%7DA_%7Bij%7D%20%26%20%5Cdots%20%26%20A%5ET_%7Bij%7D%5COmega_%7Bij%7DB_%7Bij%7D%20%26%20%5Cdots%20%26%200%20%5C%5C%0A%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%5C%5C%0A0%20%26%20%5Cdots%20%26%20B%5ET_%7Bij%7D%5COmega_%7Bij%7DA_%7Bij%7D%20%26%20%5Cdots%20%26%20B%5ET_%7Bij%7D%5COmega_%7Bij%7DB_%7Bij%7D%20%26%20%5Cdots%20%26%200%20%5C%5C%0A%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%5C%5C%0A0%20%26%20%5Cdots%20%26%200%20%26%20%5Cdots%20%26%200%20%26%20%5Cdots%20%26%200%20%0A%5Cend%7Bbmatrix%7D%0A" alt="
H_{ij} = J_{ij}^T \Omega_{ij} J_{ij} = 
\begin{bmatrix} 0 \\ \dots \\ A^T_{ij} \\ \vdots \\ B^T_{ij} \\ \vdots \\ 0 \end{bmatrix} 
\Omega_{ij} \begin{bmatrix} 0 &amp; \dots &amp; A_{ij} &amp; \dots &amp; B_{ij} &amp; \dots &amp; 0 \end{bmatrix} \\=\begin{bmatrix} 
0 &amp; \dots &amp; 0 &amp; \dots &amp; 0 &amp; \dots &amp; 0 \\
\vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots \\
0 &amp; \dots &amp; A^T_{ij}\Omega_{ij}A_{ij} &amp; \dots &amp; A^T_{ij}\Omega_{ij}B_{ij} &amp; \dots &amp; 0 \\
\vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots \\
0 &amp; \dots &amp; B^T_{ij}\Omega_{ij}A_{ij} &amp; \dots &amp; B^T_{ij}\Omega_{ij}B_{ij} &amp; \dots &amp; 0 \\
\vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots &amp; \dots &amp; \vdots \\
0 &amp; \dots &amp; 0 &amp; \dots &amp; 0 &amp; \dots &amp; 0 
\end{bmatrix}
" />

<img src="https://tex.s2cms.ru/svg/b_%7Bij%7D%20%3D%20J_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20e_%7Bij%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%20%5Cvdots%20%5C%5C%20A%5ET_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%20B%5ET_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%200%20%5Cend%7Bbmatrix%7D%0A%5COmega_%7Bij%7D%20e_%7Bij%7D%20%3D%20%0A%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%20%5Cvdots%20%5C%5C%20A%5ET_%7Bij%7D%20%5COmega_%7Bij%7D%20e_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%20B%5ET_%7Bij%7D%20%5COmega_%7Bij%7D%20e_%7Bij%7D%20%5C%5C%20%5Cvdots%20%5C%5C%200%20%5Cend%7Bbmatrix%7D" alt="b_{ij} = J_{ij}^T \Omega_{ij} e_{ij} = \begin{bmatrix} 0 \\ \vdots \\ A^T_{ij} \\ \vdots \\ B^T_{ij} \\ \vdots \\ 0 \end{bmatrix}
\Omega_{ij} e_{ij} = 
\begin{bmatrix} 0 \\ \vdots \\ A^T_{ij} \Omega_{ij} e_{ij} \\ \vdots \\ B^T_{ij} \Omega_{ij} e_{ij} \\ \vdots \\ 0 \end{bmatrix}" />

2 equations above shows that edge connecting node <img src="https://tex.s2cms.ru/svg/i" alt="i" /> to node <img src="https://tex.s2cms.ru/svg/j" alt="j" /> only contributes to entries corresponding to these nodes in both <img src="https://tex.s2cms.ru/svg/H_%7Bij%7D" alt="H_{ij}" /> and <img src="https://tex.s2cms.ru/svg/b_%7Bij%7D" alt="b_{ij}" />.

## 3. Building the linearized system

With the structure of the linearized cost function dervied above, this section shows how <img src="https://tex.s2cms.ru/svg/H" alt="H" /> and <img src="https://tex.s2cms.ru/svg/b" alt="b" /> are constructed by iterating through all edge of the pose graph.

### 3.1 Computing the Jacobian

Consider an arbitrary edge connecting node <img src="https://tex.s2cms.ru/svg/i" alt="i" /> to node <img src="https://tex.s2cms.ru/svg/j" alt="j" />, as mentioned in Sec.1, the contribution to cost function of this edge can becomputed according to pose-pose constraint or pose-landmark constraint.

If the edge is induced by a pose-pose constraint, the corresponding Jacobian is 

<img src="https://tex.s2cms.ru/svg/%20A_%7Bij%7D%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_i%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%0A-R_%7Bij%7D%5ET%20R_i%5ET%20%26%20R_%7Bij%7D%5ET%20%5Cbegin%7Bbmatrix%7D-s%5Ctheta_i%20%26%20c%5Ctheta_i%20%5C%5C%20-c%5Ctheta_i%20%26%20-s%5Ctheta_i%5Cend%7Bbmatrix%7D%20%5Cleft(t_j%20-%20t_i%5Cright)%20%5C%5C%0A0_%7B1%20%5Ctimes%202%7D%20%26%20-1%20%5Cend%7Bbmatrix%7D" alt=" A_{ij} = \frac{\partial e_{ij}(\textbf{x})}{\partial \textbf{x}_i} = \begin{bmatrix}
-R_{ij}^T R_i^T &amp; R_{ij}^T \begin{bmatrix}-s\theta_i &amp; c\theta_i \\ -c\theta_i &amp; -s\theta_i\end{bmatrix} \left(t_j - t_i\right) \\
0_{1 \times 2} &amp; -1 \end{bmatrix}" />

<img src="https://tex.s2cms.ru/svg/B_%7Bij%7D%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_j%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%0AR_%7Bij%7D%5ET%20R_i%5ET%20%26%200_%7B2%20%5Ctimes%201%7D%20%5C%5C%0A0_%7B1%20%5Ctimes%202%7D%20%26%201%20%0A%5Cend%7Bbmatrix%7D" alt="B_{ij} = \frac{\partial e_{ij}(\textbf{x})}{\partial \textbf{x}_j} = \begin{bmatrix}
R_{ij}^T R_i^T &amp; 0_{2 \times 1} \\
0_{1 \times 2} &amp; 1 
\end{bmatrix}" />

If the edge is induced by a pose-landmark constraint, the error's Jacobian is

<img src="https://tex.s2cms.ru/svg/%20A_%7Bij%7D%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20%5Ctextbf%7Bx%7D_i%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%0A-R_i%5ET%20%26%20%26%5Cbegin%7Bbmatrix%7D-s%5Ctheta_i%20%26%20c%5Ctheta_i%20%5C%5C%20-c%5Ctheta_i%20%26%20-s%5Ctheta_i%5Cend%7Bbmatrix%7D%5Cleft(m_j%20-%20t_i%5Cright)%20%0A%5Cend%7Bbmatrix%7D" alt=" A_{ij} = \frac{\partial e_{ij}(\textbf{x})}{\partial \textbf{x}_i} = \begin{bmatrix}
-R_i^T &amp; &amp;\begin{bmatrix}-s\theta_i &amp; c\theta_i \\ -c\theta_i &amp; -s\theta_i\end{bmatrix}\left(m_j - t_i\right) 
\end{bmatrix}" />

<img src="https://tex.s2cms.ru/svg/B_%7Bij%7D%20%3D%20%5Cfrac%7B%5Cpartial%20e_%7Bij%7D(%5Ctextbf%7Bx%7D)%7D%7B%5Cpartial%20m_j%7D%20%3D%20R_i%5ET" alt="B_{ij} = \frac{\partial e_{ij}(\textbf{x})}{\partial m_j} = R_i^T" />

### 3.2 Update Coefficient vector and Hessian

Because of the sparseness of the Jacobian, an edge connecting node <img src="https://tex.s2cms.ru/svg/i" alt="i" /> to node <img src="https://tex.s2cms.ru/svg/j" alt="j" /> only contributes to some certaint blocks in <img src="https://tex.s2cms.ru/svg/b" alt="b" /> and <img src="https://tex.s2cms.ru/svg/H" alt="H" /> which corresponding to index <img src="https://tex.s2cms.ru/svg/i" alt="i" /> and <img src="https://tex.s2cms.ru/svg/j" alt="j" />

The coefficient vector <img src="https://tex.s2cms.ru/svg/b" alt="b" /> written in block form as 

<img src="https://tex.s2cms.ru/svg/b%20%3D%20%5Cbegin%7Bbmatrix%7Db_1%5ET%20%26%20b_2%5ET%20%26%20%5Cdots%20%26%20b_n%5ET%5Cend%7Bbmatrix%7D" alt="b = \begin{bmatrix}b_1^T &amp; b_2^T &amp; \dots &amp; b_n^T\end{bmatrix}" />

The 2 blocks get updated by edge <img src="https://tex.s2cms.ru/svg/ij" alt="ij" /> are 

<img src="https://tex.s2cms.ru/svg/%5Cbegin%7Bmatrix%7D%0Ab_i%20%2B%3D%20A_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20e_%7Bij%7D%0A%26%26%26%0Ab_j%20%2B%3D%20B_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20e_%7Bij%7D%20%0A%5Cend%7Bmatrix%7D" alt="\begin{matrix}
b_i += A_{ij}^T \Omega_{ij} e_{ij}
&amp;&amp;&amp;
b_j += B_{ij}^T \Omega_{ij} e_{ij} 
\end{matrix}" />

The Hessian <img src="https://tex.s2cms.ru/svg/H" alt="H" /> writen in block form as 

<img src="https://tex.s2cms.ru/svg/H%20%3D%20%5Cbegin%7Bbmatrix%7D%0AH%5E%7B11%7D%20%26%20H%5E%7B12%7D%20%26%20%5Cdots%20%26%20H%5E%7B1n%7D%20%5C%5C%0AH%5E%7B21%7D%20%26%20H%5E%7B22%7D%20%26%20%5Cdots%20%26%20H%5E%7B2n%7D%20%5C%5C%0A%5Cvdots%20%26%20%5Cvdots%20%26%20%5Cdots%20%26%20%5Cvdots%20%5C%5C%0AH%5E%7Bn1%7D%20%26%20H%5E%7Bn2%7D%20%26%20%5Cdots%20%26%20H%5E%7Bnn%7D%20%0A%5Cend%7Bbmatrix%7D" alt="H = \begin{bmatrix}
H^{11} &amp; H^{12} &amp; \dots &amp; H^{1n} \\
H^{21} &amp; H^{22} &amp; \dots &amp; H^{2n} \\
\vdots &amp; \vdots &amp; \dots &amp; \vdots \\
H^{n1} &amp; H^{n2} &amp; \dots &amp; H^{nn} 
\end{bmatrix}" />

The 4 blocks of <img src="https://tex.s2cms.ru/svg/H" alt="H" /> get updated by edge <img src="https://tex.s2cms.ru/svg/ij" alt="ij" /> are

<img src="https://tex.s2cms.ru/svg/%5Cbegin%7Bmatrix%7D%0AH%5E%7Bii%7D%20%2B%3D%20A_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20A_%7Bij%7D%20%26%26%20H%5E%7Bij%7D%20%2B%3D%20A_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20B_%7Bij%7D%20%5C%5C%5C%5C%0AH%5E%7Bji%7D%20%2B%3D%20B_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20A_%7Bij%7D%20%26%26%20H%5E%7Bjj%7D%20%2B%3D%20B_%7Bij%7D%5ET%20%5COmega_%7Bij%7D%20B_%7Bij%7D%20%0A%5Cend%7Bmatrix%7D" alt="\begin{matrix}
H^{ii} += A_{ij}^T \Omega_{ij} A_{ij} &amp;&amp; H^{ij} += A_{ij}^T \Omega_{ij} B_{ij} \\\\
H^{ji} += B_{ij}^T \Omega_{ij} A_{ij} &amp;&amp; H^{jj} += B_{ij}^T \Omega_{ij} B_{ij} 
\end{matrix}" />

## 4. Result

The result of applying Least-square SLAM on three datasets are shown below.

![alt text][pose_pose_dataset]

Fig.4 Applying LS-SLAM on simulation dataset

![alt text][intel_dataset]

Fig.5 Applying LS-SLAM on intel dataset

![alt text][dlr_dataset]

Fig.6 Applying LS-SLAM on DLR dataset
