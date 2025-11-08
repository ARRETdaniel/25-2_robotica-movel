Random Force based Algorithm for Local Minima
Escape of Potential Field Method
Jinseok Lee, Yunyoung Nam and Sangjin Hong
Department of Electrical and Computer Engineering
Mobile System Design Laboratory
Stony Brook University - SUNY
Stony Brook, NY, USA
{jinlee, yynam, snjhong}@ece.sunysb.edu
Abstract— We address a new inherent limitation of potential
field methods, which is symmetrically aligned robot-obstacle-goal
(SAROG). The SAROG involves one critical risk of local minima
trap. For dealing with the problem, we investigate the way how
the local minima trap is recognized, and present our random force
algorithm. The force algorithm has two categories of random unit
total force (RUTF) and random unit total force with repulsion
removal (RUTF-RR) which are selected based on the conditions
of a robot, an obstacle and a goal.
I. I NTRODUCTION
For autonomous robot navigation, the potential field meth-
ods have been widely adopted due to its simplicity and
mathematical elegance [1][2]. However, the method has some
inherent limitations such as local minima trap. The trap
situations occur when a robot runs into a dead end, and a
robot never reach a goal [3]. To deal with the issue, tn the
previous studies, [4] used harmonic functions with Laplace’s
equation for eliminating the local minima. The method effec-
tively obviates the local minima problem with the Laplace
equation. [5] eliminated the local minima by filling up the
entire local minima region. Such new potential functions have
been actively proposed [3][6]. Together with the new potential
functions, multi-potential functions were also considered [7].
In the method, when a local minima is found in one potential
field, another potential map with different resolution ignores
the local minima.
In this paper, we address the new limitation of the potential
field methods, where a robot, an obstacle and a goal are
symmetrically aligned. Such order placement is one of the
most challenging issues of local minima trap. None of existing
algorithms cannot solve the problem. Recently, the method
using virtual obstacle [8][9] has been widely used for local
minima escape. However, in the method, the positions and
shapes of a robot and an obstacle are non-aligned and/or
asymmetric. When the virtual obstacles symmetrically exert
the repulsive forces on a robot, and a goal is positioned aligned
with the robot and the obstacle, the robot moves back and forth
only by endlessly falling into the local minima trap. We refer
to the problem of symmetrically aligned robot-obstacle-goal
as SAROG. Fig. 1 illustrates the local minima trap problem
on SAROG. For simplicity, the problems are depicted by
representing a robot, an obstacle and a goal with a point mass
in two-dimension coordinates. In case of the local minima trap,
a robot moves back and forth between the two positions A and
B. In the position A, the attractive force is stronger than the
repulsive force. For dealing with the problems of local minima
trap on SAROG, we propose new algorithm by using random
forces.
robot
obstacle
goal
local minima trap
(oscillation)
A
B
Fig. 1. Illustration of the local minima trap on on SAROG
The remainder of this paper has 5 sections. Section II briefly
explains the potential field method and describes problems
corresponding to SAROG. In Section III, we propose new
algorithm for local minim trap escape. In the algorithm,
random unit total force (RUTF) and random unit total force
with repulsion removal (RUTF-RR) are presented. Section IV
verifies the proposed algorithm using WiRobot X80. Finally,
our contribution is summarized in Section V.
II. P OTENTIAL F IELD M ETHOD AND SAROG P ROBLEM
A. Potential Field Methods
A robot, an obstacle and a goal are represented by a point
mass in two-dimension coordinates. Given a space with size
Xs × Ys, each position is denoted by p = [x y]T , where
0 ≤ x ≤ Xs and 0 ≤ y ≤ Ys. Each position of a robot, an
obstacle and a goal are denoted by pr = [xr yr]T , po =
[xo yo]T and pg = [xg yg ]T .
In the potential field method, an attractive potential is
defined as a function of the relative distance between a robot
and a goal while a repulsive potential is defined as a function
of the relative distance between a robot and an obstacle. The
two potential functions are commonly expressed as [2][10]
Uatt(p) = catt · (ρ(p, pg ))m , (1)
978-1-4244-7815-6/10/$26.00 c©2010 IEEE ICARCV2010
2010 11th Int. Conf. Control, Automation, Robotics and Vision
Singapore, 7-10th December 2010
827
Authorized licensed use limited to: UNIVERSIDADE FEDERAL DE MINAS GERAIS. Downloaded on November 07,2025 at 18:32:56 UTC from IEEE Xplore. Restrictions apply.
Frep(pr )
Fatt(pr )
Ftot (pr )
goal
obstacle
robot
pg
pr
po
o
Fctot(pr )
(a) Ftot(pr ) and Fc
tot(pr )
Frep(pr )
Fatt(pr)
pg
po
pr
F>0
F<0
s r•Ts
(b) Effect of sr ˙Ts
Fig. 2. The potential field method and its problem description
Urep(p) =
{
crep ·
( 1
ρ(p,po ) − 1
ρ0
)n
if ρ(p, po) ≤ ρ0
0 if ρ(p, po) > ρ0 ,
(2)
where catt and crep are constant values for an attractive
potential and a repulsive potential. ρ(p1, p2)=||p1, p2|| is the
shortest distance between p1 and p2. ρ0 is a positive constant
denoting the distance influence of an obstacle.
The corresponding attractive force and repulsive force are
then given by the negative gradient of each attractive potential
and repulsive potential as
Fatt(p) = −m · catt · (ρ(p, pg ))m−1 · ∇ρ(p, pg), (3)
Frep(p) =
⎧
⎪⎪⎨
⎪⎪⎩
n · crep ·
( 1
ρ(p,po) − 1
ρ0
)n−1
·
( 1
ρ(p,po)
)2
· ∇ρ(p, po) if ρ(p, po) ≤ ρ0
0 if ρ(p, po) > ρ0 ,
(4)
where ∇ρ(p, po) and ∇ρ(p, pg) are two unit vectors pointing
from po to p and from pg to p, respectively.
The total force applied to each position p is the sum
of the attractive force and the repulsive force as Ftot(p) =
Fatt(p) + Frep(p), which determines the robot direction and
speed for reaching a goal with an obstacle avoidance. As
mentioned in the previous section, we consider that a robot
moves with constant speed. Thus, the total force Ftot(p) can
be re-formulated as Fc
tot(p) = Ftot (p)
|Ftot (p)| = Fatt (p)+Frep (p)
|Fatt (p)+Frep (p)| ,
where Fc
tot is the total force in the condition of constant robot
speed. Then, given the speed sr (m/s) and sampling time
Ts(s), a robot moves sr · Ts with the direction of Fc
tot(p)
or (Fatt(p) + Frep(p))/(|Fatt(p) + Frep(p)|) every Ts. Fig.
2(a) illustrates the forces Ftot(pr ) and Fc
tot(pr ) onto a robot
by addition of the attractive force Fatt(pr ) and the repulsive
force Frep(pr ). Throughout this paper, we call Fc
tot a unit total
force. Similarly, we call Fc
att and Fc
rep a unit attractive force
and a unit repulsive force.
B. Problem Description
When |Fatt(pr )| < |Frep(pr)|, a robot moves away from a
goal and an obstacle until |Fatt(pr )| > |Frep(pr )|. Then the
robot moves back toward an obstacle and a goal, and finally
oscillates between two positions. The local minima problem
condition is summarized as
|Fatt(pr )| > |Frep(pr )|, (5)
ρ(pr, po) > sr · Ts, (6)
∇ρ(pr, po) = ∇ρ(pr, pg). (7)
The local minima problem on the condition of constant
robot speed and SAROG have not been addressed yet, and it
should be taken into account to deal with the local minima trap
(oscillation). However, a question arises: when a robot is not
initially aligned with an obstacle and a goal, the condition of
SAROG is negligible. However, the SAROG occurs even when
a robot, an obstacle and a goal are not initially aligned. As
illustrated in Fig. 3, consider that a robot is initially positioned
at (1.0m,1.0m), an obstacle is positioned at (5.0m,5.0m), and a
goal is positioned at (5.4m,4.6m). A robot speed sr is √2m/s,
and sampling time Ts is 1 second. In addition, we set up m=2,
n=2, ρ=1, catt=0.5 and crep=5. Throughout this paper, unless
otherwise noted, sr , Ts, m, n, ρ, catt and crep are as above.
As shown in Fig. 3(a), at the sampling time 0, a robot, an
obstacle and a goal are not initially aligned. However, when a
robot moves closer to a goal, a robot finally oscillates between
two points A and B by not reaching a goal as illustrated in
Fig. 3(b). Thus, the SAROG may occur when an obstacle and
a goal closely positioned even though the initial positions of
a robot, an obstacle and a goal are not aligned.
III. L OCAL M INIMA E SCAPE A LGORITHM
A. Random Force Algorithm for Local Minima Escape
1) Recognition of Local Minima Trap: On the conditions
of constant speed and SAROG, the local minima problem is
categorized into two cases. Fig. 4 illustrates the two cases
of local minima trap. One case is that a robot is trapped in
local minima before arriving a goal (i.e. oscillating across
non-goal area). The other case is that a robot is trapped after
arriving a goal (i.e. oscillating across a goal). Thus, in order
to deal with the the local minima trap, a robot should first
recognize whether it is trapped across a goal or non-goal area.
As illustrated in Fig. 4(a) when a robot is trapped across a
non-goal area, the attractive forces of positions A and B are
formed with same direction, but the total forces of the two
positions are formed with opposite direction. On the other
hand, as illustrated in Fig. 4(b) when a robot is trapped across a
goal, both the attractive forces and the total forces of positions
A and B are formed with opposite direction. Thus, a robot is
trapped across a non-goal when the conditions are formed
Fc
tot(A) = −Fc
tot(B) and Fc
att(A) = Fc
att(B), (8)
and a robot is trapped across a goal when conditions are formed
Fc
tot(A) = −Fc
tot(B) and Fc
att(A) = −Fc
att(B). (9)
On the local minima condition of (9), a robot should find the
path to reach the exact goal. For a solution, [11] proposed new
potential functions for the problem named goals nonreachable
with obstacles nearby (GNRON).
828
Authorized licensed use limited to: UNIVERSIDADE FEDERAL DE MINAS GERAIS. Downloaded on November 07,2025 at 18:32:56 UTC from IEEE Xplore. Restrictions apply.
goal
(a) At time-instant 0
A
B
(b) From time-instant 13
Fig. 3. The SAROG based local minima occurs even when a robot, an obstacle and a goal are not initially aligned
obstacle goal
A
B
robot
(a) A robot trapped be-
fore arriving a goal.
obstacle
goal
A
B
robot
(b) A robot trapped
after arriving a goal.
Fig. 4. Two cases of local minima trap on SAROG and constant robot speed
2) Random Force Algorithm for Local Minima Across Non-
Goal: On the condition of (8), a robot should find the path to
escape the local minima. For the path, we use a random unit
total force (RUTF). The RUTF determines the robot direction
and the robot escapes the local minima. After the RUTF
exertion, the original potential force is continuously used until
the condition of (8) arises. Thus, the random force algorithm
alternates potential based motions with the random motion.
Fig. 5 shows the effect of the RUTF based algorithm.
A robot starts to move from (1.0m, 1.0m) with an obstacle
positioned at (3.0m, 3.0m) and a goal positioned at (4.0m,
4.0m). The robot moves toward the goal of point (4.0m, 4.0m)
with sr = 0.4m/s and Ts = 1s. When the forces to exerting
the robot are satisfied with (8), a robot recognizes the local
minima across non-goal. Then, we exert a random unit total
force (RUTF) onto a robot as shown in Fig. 5(a). The RUTF
helps a robot escape from the oscillation. After escaping from
the oscillation, the robot continuously moves toward the goal
(4.0m, 4.0m) by potential based forces as shown in Fig. 5(b).
However, the RUTF based local minima escape is not
applicable when a robot is positioned closer to a goal than
an obstacle: the placement order is robot-goal-obstacle. To
illustrate the case, consider that a robot is positioned at
(1.0m,1.0m), an obstacle is positioned at (3.4m,3.4m) and a
goal is positioned at (3.0m,3.0m) as shown in Fig. 6(a). From
the time-instants 0 to 6, a robot moves close to a goal. At time-
instant 6, the local minima across non-goal is satisfied with (8),
and the RUTF exerts onto a robot resulting in the movement
as shown in Fig. 6(a). After the robot escapes from the local
minima, the robot continuously moves by potential based
forces as shown in Fig. 6(b) through Fig. 6(d), where the robot
moves back to the local minima. That is, the RUTF based local
minima escape is applicable only when ρ(pr , pg) > ρ(pr, po).
Otherwise, a robot moves back to previous local minima as
shown in Fig. 6(d).
Thus, in the case of ρ(pr, pg) < ρ(pr , po), we use a
potential function with repulsion removal as
Fc
tot(p) = Fc
att(p)
|Fc
att(p)| . (10)
That is, once the potential forces and the distance conditions
are both satisfied with (8) and ρ(pr , pg) < ρ(pr , po), the
RUTF changes robot direction and the potential force of (10)
exerts onto a robot. The RUTF and the potential function with
removal (RUTF-RR) algorithm derives a robot to reach a goal
by removing a repulsion force since the movement back to
previous local minima after RUTF is from the repulsive force
existence. The attractive force exertion of (10) continues until
a robot arrives at a goal. When the RUTF-RR is used, the robot
collision issue should be considered as well. Since the robot
moves with only attractive force, it is possible for the robot to
collide with an obstacle. The collision occurs more frequently
when the size of a robot and/or an obstacle increases, and
the distance between a goal and an obstacle becomes shorter.
In order to investigate the anti-collision condition, Fig. 7(a)
829
Authorized licensed use limited to: UNIVERSIDADE FEDERAL DE MINAS GERAIS. Downloaded on November 07,2025 at 18:32:56 UTC from IEEE Xplore. Restrictions apply.
starting point
obstacle
destination
robot
goal
(a) RUTF exertion (b) After RUTF
Fig. 5. Local minima escape using RUTF
starting point
obstacle
destination
(a) time-instant 0 to 7 (b) time-instant 0 to 10 (c) time-instant 0 to 12 (d) time-instant 0 to 20
Fig. 6. RUTF algorithm limitation: Even though the RUTF exerts onto a robot, the robot eventually moves back to the local minima of SAROG.
illustrates the placement and the size of a robot, a goal and
an obstacle. Given the robot position Pr , the RUTF should
be generated in the collision free area. Then, after the RUTF
exertion to the collision free area, the collision is prevented
even with the attraction force only. Fig. 7(a) is re-illustrated
in Fig. 7(b) to find the anti-collision condition. The angles θ1
and θ2 are formulated as θ1 = θ2 = csc
( rr +ro
dv
)
Then, the
shortest distance dw between a robot and the shaded region
boundary is expressed as dw = du · sin θ1. In addition, the
RUTF angle θ(Fc
tot) is
θ(Fc
tot) = π − θ1. (11)
In order to move inside the shaded region with speed sr , the
robot should keep moving with the force direction ±(π − θ1)
for τ time-instants, where τ =
⌈ dw
sr
⌉
. The sign ± of the force
direction is randomly assigned. After τ time-instants, a robot
keeps moving by the attraction only.
IV. A LGORITHM V ERIFICATION AND A NALYSIS
A. Simulation Setup
For the algorithm verification, we used the WiRobot X80 as
shown in Fig. 8. The X80 underlies technology evolved from
Dr Robot Distributed Computation Robotic Architecture, origi-
nally developed for Dr Robot Humanoid Robot. It is developed
for fast and strong motion, while itself remaining lightweight
and nimble. The wheel-based platform is with two 12V DC
motors each supply 22kg·cm of torque to the 18 cm wheels,
yielding a top speed in excess of 1 m/s. Two high-resolution
with 1200 count per wheel cycle quadrature encoders mounted
on each wheel provide high-precision measurement and control
of wheel movement. For estimating the distance between a
robot and an obstacle, we equipped the robot with three
ultrasonic range sensor modules of DUR5200. The range
sensor detects an obstacle within 3.4 meters. The distance data
is precisely presented by the time interval between the time-
instant when the measurement is enabled and the time-instant
when the echo signal is received. By using the ultrasonic range
sensor modules, the robot recognizes its near obstacle position
up to 3.4 meters. For the proposed algorithm verification on
the restricted two conditions of SAROG and robot constant
speed. we set up the robot, obstacles and the goal as in Fig.
9. Initially, goal 1 is enabled while goal 2 and goal 3 are
disabled. After the robot reaches goal 1, goal 2 only is enabled.
Finally, after the robot reaches goal 2, goal 3 only is enabled.
830
Authorized licensed use limited to: UNIVERSIDADE FEDERAL DE MINAS GERAIS. Downloaded on November 07,2025 at 18:32:56 UTC from IEEE Xplore. Restrictions apply.
ro
obstacle
goal
rr
rr
r r
r r
rr
Pr
robot
collision free area
(a) The placement and the size of a robot, a goal
and an obstacle
Pr
1
2
du d v Po
rr +ro
rr +ro
(Fctot)
original robot
orientation
collision free area
d w
(b) Geometry for dw and θ(Fc
tot)
Fig. 7. Illustration of the anti-collision condition
ultrasonic range sensor modules
(DUR5200)
Fig. 8. WiRobot X80: three ultrasonic range sensor modules of DUR5200
are attached for distance estimation.
For the navigation to goal 1, the robot has the problem on
SAROG; and thus, will be trapped on local minim by using
potential field method only. In the local minima trap, we will
show the RUTF will solve the problem. For the navigation to
goal 3, the robot also has the problem on SAROG; and thus
will be trapped on local minima again. However, in the case,
the RUTF cannot solve the problem; the robot moves back to
original local minima after RUTF. In the local minima trap,
we will show the RUTF-RR deals with the problem.
B. Results and Discussion
As depicted in Fig. 9, X80 traveled from Start to three
goals; goal 1 positioned at (2.5m,4.0m), goal 2 positioned
at (5.5m,7.0m) and goal 3 positioned at (8.5m,8.0m). Each
obstacle is positioned at (4.0m,2.5m), (3.5m,6.0m) and
(8.0m,8.5m) with ro = 0.4375m and ρo = 1.2m. The
robot starts from the position (5.5m,1.0m) with rr = 0.25m,
Ts = 1s and sr = 0.5m/s. With goal 1 only enabled, the
robot first moved toward goal 1 with potential functions and
kept moving until the condition of (8) is satisfied at time 2
(s). At time 2 (s), the robot compared the distance condition,
which was ρ(pr , pg ) > ρ(pr, po), and RUTF instead of the
potential functions determined robot direction for the local
minima escape. From 3 (s) to 13 (s), the robot moved toward
goal 1
goal 2
goal 3
start
obstacle
obstacle
obstacle
x (m)
y (m)
RUTF
RUTF-RR
Fig. 9. WiRobot X80 traveled from Start to goal 1, goal 2 and goal 3 in
order with RUTF and RUTF-RR algorithms.
goal 1 by using the potential field, and finally arrived at goal 1.
Once the robot arrived at goal 1, goal 1 was disabled and only
goal 2 was enabled. We set the starting time from goal 1 to
zero. From goal 1 to goal 2, any local minima problem did not
occur. Note until time 2 (s), the robot has the potential function
involved with attraction force only. From time 2 (s) to 6 (s),
the potential function involved both the attraction force and the
repulsion force since the robot positioned within the obstacle
influence. From 6 (s) to 9 (s), the potential function involved
with attraction force only and the robot finally arrived at goal 2.
Similarly, once the robot arrived at goal 2, goal 2 was disabled
and only goal 3 was enabled. We also set the starting time from
goal 2 to zero. From goal 2 the robot moved toward goal 3 with
potential functions and kept moving until the condition of (8)
is satisfied at time 15 (s). At time 15 (s), the robot compared
the distance condition, which was ρ(pr, pg ) < ρ(pr , po), and
RUTF-RR instead of the potential functions determined robot
direction for the local minima escape, and finally let the robot
arrived at goal 3 with τ = 1.
831
Authorized licensed use limited to: UNIVERSIDADE FEDERAL DE MINAS GERAIS. Downloaded on November 07,2025 at 18:32:56 UTC from IEEE Xplore. Restrictions apply.
V. C ONCLUSION
We have described new problem of symmetrically aligned
robot-obstacle-goal (SAROG) with using potential field meth-
ods. For dealing with the corresponding the critical issue of
local minima trap, random force based algorithms have been
proposed and they have been verified using WiRobot X80 with
three ultrasonic range sensor modules.
