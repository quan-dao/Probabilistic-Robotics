[//]: # (Image References)
[bob_env]: ./images/bob_env.png
[line_fitting]: ./images/line_fitting.png
[ekf_local]: ./images/EKF_localization.gif
[just_odometry]: ./images/just_odometry.gif

# 1. Intro
This implementation of EKF Localization is forked from [Edx/AMRx-Exercise-4](https://www.edx.org/course/autonomous-mobile-robots-2)

The EKF Localization algorithm is implemented for a differential drive robot equipped with a ring of rangefinders, operated in the environment shown in Fig.1. It's worth to notice that the sensory output is a set of 2D points on the plane defined by the rangefinders ring.

![alt text][bob_env]
Fig.1 Robot in simulation environment

Since the EKF Localization can only address the position tracking problem, the initial position of the robot and its uncertainty are assumed to be known. In addition, the environment map is also provided.

To fully define robot pose in a 2D environment, robot state vector <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> is chosen to be its position <img src="https://tex.s2cms.ru/svg/(x_t%2C%20y_t)" alt="(x_t, y_t)" /> and its heading orientation <img src="https://tex.s2cms.ru/svg/%5Ctheta_t" alt="\theta_t" /> with respect to global frame.  

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t%20%3D%20%5Cbegin%7Bbmatrix%7Dx_t%20%26%20y_t%20%26%20z_t%20%5Cend%7Bbmatrix%7D%5ET" alt="\textbf{x}_t = \begin{bmatrix}x_t &amp; y_t &amp; z_t \end{bmatrix}^T" /> 

From the raw sensory output (a set of 2D points), several lines are extracted for localization; hence the map <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BM%7D" alt="\textbf{M}" /> is the set of every line <img src="https://tex.s2cms.ru/svg/m_i%20(i%20%3D%200%2C%20...%2C%20n-1)" alt="m_i (i = 0, ..., n-1)" /> in the environment.  

<img src="https://tex.s2cms.ru/svg/M%20%3D%20%5Cbegin%7BBmatrix%7Dm_0%2C%20m_1%2C%20%5Cdots%2C%20m_%7Bn%20-1%7D%5Cend%7BBmatrix%7D" alt="M = \begin{Bmatrix}m_0, m_1, \dots, m_{n -1}\end{Bmatrix}" />

The EKF Localization algorithm is comprised of 3 steps: 
1. Motion Prediction 
2. Measurement Prediction
3. Estimation

# 2. Motion Prediction
Given the current pose of robot <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_%7Bt%20-%201%7D" alt="\textbf{x}_{t - 1}" /> and the control input <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" />, new robot pose <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> can be predicted by the motion model. In this example, the <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" /> is robot odometry - displacement of left and right wheel.

<img src="https://tex.s2cms.ru/svg/u_t%20%3D%20%5Cbegin%7Bbmatrix%7D%5CDelta%20s_l%20%26%20%5CDelta%20s_r%20%5Cend%7Bbmatrix%7D%5ET" alt="u_t = \begin{bmatrix}\Delta s_l &amp; \Delta s_r \end{bmatrix}^T" /> 

Using the kinematic model of differential drive robot, the motion model is established as

<img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D_t%20%3D%20f(%5Ctextbf%7Bx%7D_%7Bt-1%7D%2C%20u_t)%20%3D%20%5Ctextbf%7Bx%7D_%7Bt-1%7D%20%2B%20%5Cbegin%7Bbmatrix%7D%20%0A%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccdot%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccdot%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7Bb%7D%20%0A%5Cend%7Bbmatrix%7D" alt="\hat{\textbf{x}}_t = f(\textbf{x}_{t-1}, u_t) = \textbf{x}_{t-1} + \begin{bmatrix} 
\frac{\Delta s_l + \Delta s_r}{2} \cdot \cos\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
\frac{\Delta s_l + \Delta s_r}{2} \cdot \sin\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
\frac{\Delta s_r - \Delta s_l}{b} 
\end{bmatrix}" /> 

<img src="https://tex.s2cms.ru/svg/b" alt="b" /> hereupon is the distance between left and right wheel. Assume the covariance of the control input <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" /> has the form of 

<img src="https://tex.s2cms.ru/svg/Q_t%20%3D%20%5Cbegin%7Bbmatrix%7D%20k%7C%5CDelta%20s_l%7C%20%26%200%20%5C%5C%200%20%26%20k%7C%5CDelta%20s_r%7C%20%5Cend%7Bbmatrix%7D" alt="Q_t = \begin{bmatrix} k|\Delta s_l| &amp; 0 \\ 0 &amp; k|\Delta s_r| \end{bmatrix}" /> 

Here, <img src="https://tex.s2cms.ru/svg/k" alt="k" /> is a constance. Given <img src="https://tex.s2cms.ru/svg/Q_t" alt="Q_t" /> calculated by Eq.5 and the covariance of the current pose <img src="https://tex.s2cms.ru/svg/P_%7Bt-1%7D" alt="P_{t-1}" />, the covariance of the new pose predicted by the motion model is

<img src="https://tex.s2cms.ru/svg/%5Chat%7BP%7D_t%20%3D%20F_x%20%5Ccdot%20P_%7Bt%20-%201%7D%20%5Ccdot%20F_x%5ET%20%2B%20F_u%20%5Ccdot%20Q_%7Bt%7D%20%5Ccdot%20F_u%5ET" alt="\hat{P}_t = F_x \cdot P_{t - 1} \cdot F_x^T + F_u \cdot Q_{t} \cdot F_u^T" /> 

In equation above, <img src="https://tex.s2cms.ru/svg/F_x%2C%20F_u" alt="F_{\textbf{x}}, F_u" /> are respectively the Jacobian of <img src="https://tex.s2cms.ru/svg/f(%5Ctextbf%7Bx%7D_%7Bt-1%7D%2C%20u_t)" alt="f(\textbf{x}_{t-1}, u_t)" /> with respect to <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_t" alt="\textbf{x}_t" /> and <img src="https://tex.s2cms.ru/svg/u_t" alt="u_t" />, evaluated at the value of <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7Bx%7D_%7Bt-1%7D%2C%20u_t" alt="\textbf{x}_{t-1}, u_t" />.

<img src="https://tex.s2cms.ru/svg/F_x%20%3D%20I%20%2B%20%5Cbegin%7Bbmatrix%7D%0A0%20%26%200%20%26%20-%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%20%0A0%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20s_r%20-%20%5CDelta%20s_l%7D%7B2b%7D%5Cright)%20%5C%5C%0A0%20%26%200%20%26%200%0A%5Cend%7Bbmatrix%7D" alt="F_{\textbf{x}} = I + \begin{bmatrix}
0 &amp; 0 &amp; -\frac{\Delta s_l + \Delta s_r}{2} \sin\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\ 
0 &amp; 0 &amp; \frac{\Delta s_l + \Delta s_r}{2} \cos\left(\theta_{t-1} + \frac{\Delta s_r - \Delta s_l}{2b}\right) \\
0 &amp; 0 &amp; 0
\end{bmatrix}" /> 

Let <img src="https://tex.s2cms.ru/svg/%5CDelta%20%5Ctheta%20%3D%20%5Cleft(%5CDelta%20s_r%20-%20%5CDelta%20s_l%5Cright)%20%2F%20b" alt="\Delta \theta = \left(\Delta s_r - \Delta s_l\right) / b" />

<img src="https://tex.s2cms.ru/svg/F_u%20%3D%20%5Cfrac%7B1%7D%7B2%7D%5Cbegin%7Bbmatrix%7D%0A%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%2B%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%26%0A%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20-%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%5C%5C%0A%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20-%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%26%20%0A%5Csin%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%2B%20%5Cfrac%7B%5CDelta%20s_l%20%2B%20%5CDelta%20s_r%7D%7B2b%7D%20%5Ccos%5Cleft(%5Ctheta_%7Bt-1%7D%20%2B%20%5Cfrac%7B%5CDelta%20%5Ctheta%7D%7B2%7D%5Cright)%20%5C%5C%0A%5Cfrac%7B-1%7D%7Bb%7D%20%26%20%5Cfrac%7B1%7D%7Bb%7D%0A%5Cend%7Bbmatrix%7D" alt="F_u = \frac{1}{2}\begin{bmatrix}
\cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) + \frac{\Delta s_l + \Delta s_r}{2b} \sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) &amp;
\cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) - \frac{\Delta s_l + \Delta s_r}{2b} \sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) \\
\sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) - \frac{\Delta s_l + \Delta s_r}{2b} \cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) &amp; 
\sin\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) + \frac{\Delta s_l + \Delta s_r}{2b} \cos\left(\theta_{t-1} + \frac{\Delta \theta}{2}\right) \\
\frac{-1}{b} &amp; \frac{1}{b}
\end{bmatrix}" />

# 3. Measurement Prediction
## 3.1 Measurement Model
As mentioned in Sec.1, robot has a ring of rangefinders; therefore the raw measurement is a set 2D points. From this set, several lines are extracted to serve the process of feature-based localization with EKF. In this example, the extraction of line features from raw measurement is already provided. At every time instance, robot has access to set of lines <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BZ%7D_t%20%3D%20%5Cleft%5C%7Bz_t%5Ei%20%7C%20i%20%3D%200%2C%20%5Cdots%2C%20m-1%5Cright%5C%7D" alt="\textbf{Z}_t = \left\{z_t^i | i = 0, \dots, m-1\right\}" /> and their covariant matrix <img src="https://tex.s2cms.ru/svg/R_t%5Ei" alt="R_t^i" />. Here <img src="https://tex.s2cms.ru/svg/m" alt="m" /> is the number of extracted line features. Each line <img src="https://tex.s2cms.ru/svg/z_t%5Ei" alt="z_t^i" /> is parameterized by two number <img src="https://tex.s2cms.ru/svg/%5Calpha_t%5Ei" alt="\alpha_t^i" /> and <img src="https://tex.s2cms.ru/svg/r_t%5Ei" alt="r_t^i" />. 

<img src="https://tex.s2cms.ru/svg/z_t%5Ei%20%3D%20%5Cbegin%7Bbmatrix%7D%5Calpha_t%5Ei%20%26%20r_t%5Ei%5Cend%7Bbmatrix%7D%5ET" alt="z_t^i = \begin{bmatrix}\alpha_t^i &amp; r_t^i\end{bmatrix}^T" />

The definition of these numbers is shown in Fig.2.  

![alt text][line_fitting]

Fig.2 Parameterize a line  

The measurement model is to predict the obtained measurment given robot position and the environment map. The key insight for establishing the model is that each feature <img src="https://tex.s2cms.ru/svg/m%5Ej%20%3D%20%5Cbegin%7Bbmatrix%7D%5Calpha%5Ej%20%26%20r%5Ej%5Cend%7Bbmatrix%7D%5ET" alt="m^j = \begin{bmatrix}\alpha^j &amp; r^j\end{bmatrix}^T" /> of the is expressed in the world frame <img src="https://tex.s2cms.ru/svg/%5C%7B%5Ctextbf%7BW%7D%5C%7D" alt="\{\textbf{W}\}" />, while features obtained by robot's sensors are expressed in robot's body frame <img src="https://tex.s2cms.ru/svg/%5C%7B%5Ctextbf%7BR%7D%5C%7D" alt="\{\textbf{R}\}" />. As a result, to predict robot measurement, all environment's features need to be transformed from <img src="https://tex.s2cms.ru/svg/%5C%7B%5Ctextbf%7BW%7D%5C%7D" alt="\{\textbf{W}\}" /> to <img src="https://tex.s2cms.ru/svg/%5C%7B%5Ctextbf%7BR%7D%5C%7D" alt="\{\textbf{R}\}" />. Such transformation formulates the measurement model and is implemented below

<img src="https://tex.s2cms.ru/svg/%5Chat%7Bz%7D_t%5Ej%20%3D%20h%5Ej(%5Chat%7B%5Ctextbf%7Bx%7D%7D_t%2C%20m%5Ej)%20%3D%20%5Cbegin%7Bbmatrix%7D%0A%5Calpha%5Ej%20-%20%5Chat%7B%5Ctheta%7D_t%20%5C%5C%0Ar%5Ej%20-%20%5Cleft(%5Chat%7Bx%7D_t%20%5Ccdot%20%5Ccos(%5Calpha%5Ej)%20%2B%20%5Chat%7By%7D_t%20%5Ccdot%20%5Csin(%5Calpha%5Ej)%5Cright)%0A%5Cend%7Bbmatrix%7D" alt="\hat{z}_t^j = h^j(\hat{\textbf{x}}_t, m^j) = \begin{bmatrix}
\alpha^j - \hat{\theta}_t \\
r^j - \left(\hat{x}_t \cdot \cos(\alpha^j) + \hat{y}_t \cdot \sin(\alpha^j)\right)
\end{bmatrix}" />

For later usage, the Jacobian of the measurement model above is dervied
<img src="https://tex.s2cms.ru/svg/H_%7B%5Ctextbf%7Bx%7D%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%0A0%20%26%200%20%26%20-1%20%5C%5C%0A-%5Ccos%5Cleft(%5Calpha%5Ej%5Cright)%20%26%20-%5Csin%5Cleft(%5Calpha%5Ej%5Cright)%20%26%200%0A%5Cend%7Bbmatrix%7D" alt="H_{\textbf{x}}^j = \begin{bmatrix}
0 &amp; 0 &amp; -1 \\
-\cos\left(\alpha^j\right) &amp; -\sin\left(\alpha^j\right) &amp; 0
\end{bmatrix}" />

## 3.2 Measurement Association
At this point, robot have obtained 2 sets of measurement: the actual measurement <img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BZ%7D_t%20%3D%20%5Cleft%5C%7Bz_t%5Ei%20%7C%20i%20%3D%200%2C%20%5Cdots%2C%20m-1%5Cright%5C%7D" alt="\textbf{Z}_t = \left\{z_t^i | i = 0, \dots, m-1\right\}" /> and the predicted measurement <img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7BZ%7D%7D_t%20%3D%20%5Cleft%5C%7B%5Chat%7Bz%7D%5Ej_t%20%7C%20j%20%3D%200%2C%20%5Cdots%2C%20n-1%5Cright%5C%7D" alt="\hat{\textbf{Z}}_t = \left\{\hat{z}^j_t | j = 0, \dots, n-1\right\}" />. For the Kalman Filter to operate, the correspondence between acutal and predicted measurement need to be established. To this end, the Mahalanobis distance between the acutal measurement <img src="https://tex.s2cms.ru/svg/z_t%5Ei" alt="z_t^i" /> and the predicted measurement <img src="https://tex.s2cms.ru/svg/%5Chat%7Bz%7D_t%5Ej" alt="\hat{z}_t^j" /> is employed. 

To calculate the Mahalanobis distance, the innovation vector <img src="https://tex.s2cms.ru/svg/v_t%5E%7Bij%7D" alt="v_t^{ij}" /> is first computed

<img src="https://tex.s2cms.ru/svg/v_t%5E%7Bij%7D%20%3D%20z_t%5Ei%20-%20%5Chat%7Bz%7D_t%5Ej" alt="v_t^{ij} = z_t^i - \hat{z}_t^j" />

The covariant matrix of <img src="https://tex.s2cms.ru/svg/v_t%5E%7Bij%7D" alt="v_t^{ij}" /> is calculated by   

<img src="https://tex.s2cms.ru/svg/%5CSigma_%7BIN_t%7D%5E%7Bij%7D%20%3D%20R_t%5Ei%20%2B%20H_%7B%5Ctextbf%7Bx%7D%7D%20%5Ccdot%20%5Chat%7BP%7D_t%20%5Ccdot%20H_%7B%5Ctextbf%7Bx%7D%7D%5ET%20" alt="\Sigma_{IN_t}^{ij} = R_t^i + H_{\textbf{x}} \cdot \hat{P}_t \cdot H_{\textbf{x}}^T " />

Here, <img src="https://tex.s2cms.ru/svg/R_t%5Ei" alt="R_t^i" /> is the covariant matrix of <img src="https://tex.s2cms.ru/svg/z_t%5Ei" alt="z_t^i" />. This matrix is inferred from the covariant associated with the noise of all 2D points that form <img src="https://tex.s2cms.ru/svg/z_t%5Ei" alt="z_t^i" />.

The Mahalanobis distance <img src="https://tex.s2cms.ru/svg/d_t%5E%7Bij%7D" alt="d_t^{ij}" /> is 

<img src="https://tex.s2cms.ru/svg/d_t%5E%7Bij%7D%20%3D%20v_t%5E%7Bij%7D%5ET%20%5Ccdot%20%5Cleft(%5CSigma_%7BIN_t%7D%5E%7Bij%7D%5Cright)%5E%7B-1%7D%20%5Ccdot%20v_t%5E%7Bij%7D" alt="d_t^{ij} = v_t^{ij}^T \cdot \left(\Sigma_{IN_t}^{ij}\right)^{-1} \cdot v_t^{ij}" />

If this distance is smaller than a threshold, a correspondence is found. In case there are more than 1 predicted feature match with one actual feature, the predicted feature with smallest distance is chosen.

# 4. Estimation
In this step, the matched measurements are used to correct the prediction of motion model <img src="https://tex.s2cms.ru/svg/%5Chat%7B%5Ctextbf%7Bx%7D%7D%7D_t%2C%20%5Chat%7BP%7D_t" alt="\hat{\textbf{x}}}_t, \hat{P}_t" />. 

For all pairs of match actual and predicted measurement, vertically concatenate the actual measurements into vector <img src="https://tex.s2cms.ru/svg/z_t" alt="z_t" /> and the predicted measurements into <img src="https://tex.s2cms.ru/svg/%5Chat%7Bz%7D_t" alt="\hat{z}_t" />. The composite innovation vector <img src="https://tex.s2cms.ru/svg/v_t" alt="v_t" /> is

<img src="https://tex.s2cms.ru/svg/v_t%20%3D%20z_t%20-%20%5Chat%7Bz%7D_t" alt="v_t = z_t - \hat{z}_t" />

To calculate the covariant matrix of <img src="https://tex.s2cms.ru/svg/v_t" alt="v_t" /> - <img src="https://tex.s2cms.ru/svg/%5CSigma_%7BIN_t%7D" alt="\Sigma_{IN_t}" /> , vertically concatenate all <img src="https://tex.s2cms.ru/svg/H_%7B%5Ctextbf%7Bx%7D%7D%5Ej" alt="H_{\textbf{x}}^j" /> to make the composite Jacobian of measurement model <img src="https://tex.s2cms.ru/svg/H_t" alt="H_t" />; and put the covariant matrix of all actual measurement <img src="https://tex.s2cms.ru/svg/R_t%5Ei" alt="R_t^i" /> into a block diagonal matrix <img src="https://tex.s2cms.ru/svg/R_t" alt="R_t" />. Similarly to the covariant matrix of a single pair of match measurements, <img src="https://tex.s2cms.ru/svg/%5CSigma_%7BIN_t%7D" alt="\Sigma_{IN_t}" /> is calculated by 

<img src="https://tex.s2cms.ru/svg/%5CSigma_%7BIN_t%7D%20%3D%20R_t%20%2B%20H_%7B%5Ctextbf%7Bx%7D%7D%5Ccdot%20%5Chat%7BP%7D_t%20%5Ccdot%20H_%7B%5Ctextbf%7Bx%7D%7D%5ET" alt="\Sigma_{IN_t} = R_t + H_{\textbf{x}}\cdot \hat{P}_t \cdot H_{\textbf{x}}^T" />

Next, compute the Kalman gain

<img src="https://tex.s2cms.ru/svg/%5Ctextbf%7BK%7D_t%20%3D%20%5Chat%7BP%7D_t%20%5Ccdot%20H_t%5ET%20%5Ccdot%20%5CSigma_%7BIN_t%7D" alt="\textbf{K}_t = \hat{P}_t \cdot H_t^T \cdot \Sigma_{IN_t}" />

The new estimation of robot pose and its covariant matrix is 

<img src="https://tex.s2cms.ru/svg/%20%5Ctextbf%7Bx%7D_t%20%3D%20%5Chat%7B%5Ctextbf%7Bx%7D%7D_t%20%2B%20%5Ctextbf%7BK%7D_t%20%5Ccdot%20v_t%20" alt=" \textbf{x}_t = \hat{\textbf{x}}_t + \textbf{K}_t \cdot v_t " />

<img src="https://tex.s2cms.ru/svg/%20P_t%20%3D%20%5Cleft(I%20-%20%5Ctextbf%7BK%7D_t%20%5Ccdot%20H_t%20%5Cright)%20%5Ccdot%20%5Chat%7BP%7D_t%20" alt=" P_t = \left(I - \textbf{K}_t \cdot H_t \right) \cdot \hat{P}_t " />

# 5. Result
The comparison between EKF Localization and Odometry is shown below. In those figures, the grought truth and the estimated by either EKF or odometry is respectively denoted by the grey robot and the yellow robot. It can be seen that while the Odometry quickly diverse from the ground truth, EKF Localization still manages to track the true state.

![alt text][just_odometry]

Fig.3 Odometry result 

![alt text][ekf_local]

Fig.4 EKF Localization result
