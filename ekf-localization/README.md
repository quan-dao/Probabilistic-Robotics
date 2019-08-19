[//]: # (Image References)
[bob_env]: ./images/bob_env.png


# 1. Intro
The EKF Localization algorithm is implemented for a differential drive robot equipped with a ring of rangefinders, operated in the environment shown in Fig.1. It's worth to notice that the sensory output is a set of 2D points on the plane defined by the rangefinders ring.

![alt text][bob_env]
Fig.1 Robot in simulation environment

Since the EKF Localization can only address the position tracking problem, the initial position of the robot and its uncertainty are assumed to be known. In addition, the environment map is also provided.

To fully define robot pose in a 2D environment, robot state vector <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> is chosen to be its position <img src="https://tex.s2cms.ru/svg/(x_t%2C%20y_t)" alt="(x_t, y_t)" /> and its heading orientation <img src="https://tex.s2cms.ru/svg/%5Ctheta_t" alt="\theta_t" /> with respect to global frame.  

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t%20%3D%20%5Cbegin%7Bbmatrix%7Dx_t%20%26%20y_t%20%26%20z_t%20%5Cend%7Bbmatrix%7D%5ET" alt="\textbf{x}_t = \begin{bmatrix}x_t &amp; y_t &amp; z_t \end{bmatrix}^T" /> (1)

From the raw sensory output (a set of 2D points), several lines are extracted for localization; hence the map <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BM%7D" alt="\textbf{M}" /> is the set of every line <img src="https://tex.s2cms.ru/svg/m_i%20(i%20%3D%200%2C%20...%2C%20n-1)" alt="m_i (i = 0, ..., n-1)" /> in the environment.  

<img src="https://tex.s2cms.ru/svg/M%20%3D%20%5Cbegin%7BBmatrix%7Dm_0%2C%20m_1%2C%20%5Cdots%2C%20m_%7Bn%20-1%7D%5Cend%7BBmatrix%7D" alt="M = \begin{Bmatrix}m_0, m_1, \dots, m_{n -1}\end{Bmatrix}" /> (2)

# 2. Motion Model
The motion model is to predict robot new pose <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> given its current pose <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_%7Bt%20-%201%7D" alt="\textbf{x}_{t - 1}" /> and new control input <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" />. In this example, the <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" /> is robot odometry - displacement of left and right wheel.

<img src="https://tex.s2cms.ru/svg/u_t%20%3D%20%5Cbegin%7Bbmatrix%7D%5CDelta%20s_l%20%26%20%5CDelta%20s_r%20%5Cend%7Bbmatrix%7D%5ET" alt="u_t = \begin{bmatrix}\Delta s_l &amp; \Delta s_r \end{bmatrix}^T" /> (3)

Using the kinematic model of differential drive robot, the motion model is established as

<img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D_t%20%3D%20f(%5Ctextbf%7Bx%7D_%7Bt-1%7D%2C%20u_t)%20%3D%20%5Ctextbf%7Bx%7D_%7Bt-1%7D%20%2B%20%5Cbegin%7Bbmatrix%7D%20%0A%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccdot%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccdot%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7Bb%7D%20%0A%5Cend%7Bbmatrix%7D" alt="\hat{\textbf{x}}_t = f(\textbf{x}_{t-1}, u_t) = \textbf{x}_{t-1} + \begin{bmatrix} 
\frac{\Delta s_l + \Delta s_r}{2} \cdot \cos\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
\frac{\Delta s_l + \Delta s_r}{2} \cdot \sin\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
\frac{\Delta s_r - \Delta s_l}{b} 
\end{bmatrix}" /> (4)

<img src="https://tex.s2cms.ru/svg/b" alt="b" /> in Eq.4 and hereupon is the distance between left and right wheel. Assume the covariance of the control input <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" /> has the form of 

<img src="https://tex.s2cms.ru/svg/Q_t%20%3D%20%5Cbegin%7Bbmatrix%7D%20k%7C%5CDelta%20s_l%7C%20%26%200%20%5C%5C%200%20%26%20k%7C%5CDelta%20s_r%7C%20%5Cend%7Bbmatrix%7D" alt="Q_t = \begin{bmatrix} k|\Delta s_l| &amp; 0 \\ 0 &amp; k|\Delta s_r| \end{bmatrix}" /> (5)

Here, <img src="https://tex.s2cms.ru/svg/k" alt="k" /> is a constance. Given <img src="https://tex.s2cms.ru/svg/Q_t" alt="Q_t" /> calculated by Eq.5 and the covariance of the current pose <img src="https://tex.s2cms.ru/svg/P_%7Bt-1%7D" alt="P_{t-1}" />, the covariance of the new pose predicted by the motion model is

<img src="https://tex.s2cms.ru/svg/%5Chat%7BP%7D_t%20%3D%20F_x%20%5Ccdot%20P_%7Bt%20-%201%7D%20%5Ccdot%20F_x%5ET%20%2B%20F_u%20%5Ccdot%20Q_%7Bt%7D%20%5Ccdot%20F_u%5ET" alt="\hat{P}_t = F_x \cdot P_{t - 1} \cdot F_x^T + F_u \cdot Q_{t} \cdot F_u^T" /> (6)

In Eq.6, <img src="https://tex.s2cms.ru/svg/F_x%2C%20F_u" alt="F_x, F_u" /> are respectively the Jacobian of $f(\textbf{x}_{t-1}, u_t)$ with respect to <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> and <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" />, evaluated at the value of <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_%7Bt-1%7D%2C%20u_t" alt="\textbf{x}_{t-1}, u_t" />.

<img src="https://tex.s2cms.ru/svg/F_x%20%3D%20I%20%2B%20%5Cbegin%7Bbmatrix%7D%0A0%20%26%200%20%26%20-%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%20%0A0%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A0%20%26%200%20%26%200%0A%5Cend%7Bbmatrix%7D" alt="F_x = I + \begin{bmatrix}
0 &amp; 0 &amp; -\frac{\Delta s_l + \Delta s_r}{2} \sin\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\ 
0 &amp; 0 &amp; \frac{\Delta s_l + \Delta s_r}{2} \cos\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
0 &amp; 0 &amp; 0
\end{bmatrix}" /> (7)

Let <img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctheta%20%3D%20%5Cleft(%5CDelta%20s_r%20-%20%5CDelta%20s_l%5Cright)%20%2F%20b" alt="\Delta \theta = \left(\Delta s_r - \Delta s_l\right) / b" />

<img src="https://tex.s2cms.ru/svg/F_u%20%3D%20%5Cfrac%7B1%7D%7B2%7D%5Cbegin%7Bbmatrix%7D%0A%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%2B%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%26%0A%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20-%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%5C%5C%0A%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20-%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%26%20%0A%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%2B%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B-1%7D%7Bb%7D%20%26%20%5Cfrac%7B1%7D%7Bb%7D%0A%5Cend%7Bbmatrix%7D" alt="F_u = \frac{1}{2}\begin{bmatrix}
\cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) + \frac{\Delta s_l + \Delta s_r}{2b} \sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) &amp;
\cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) - \frac{\Delta s_l + \Delta s_r}{2b} \sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) \\
\sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) - \frac{\Delta s_l + \Delta s_r}{2b} \cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) &amp; 
\sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) + \frac{\Delta s_l + \Delta s_r}{2b} \cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) \\
\frac{-1}{b} &amp; \frac{1}{b}
\end{bmatrix}" />