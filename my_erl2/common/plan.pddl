Number of literals: 19
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
27% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 10.000
b (7.000 | 0.000)b (5.000 | 5.001)b (3.000 | 10.002)b (2.000 | 15.003)b (1.000 | 20.004);;;; Solution Found
; States evaluated: 9
; Cost: 25.005
; Time 0.02
0.000: (new_turn sherlock_robot wp4 wp3 wp2 wp1 wp0)  [0.001]
0.001: (move sherlock_robot wp4 wp1)  [5.000]
5.002: (move_gripper sherlock_robot wp1 hy)  [5.000]
10.003: (perceive_hint sherlock_robot wp1 hy)  [5.000]
15.004: (check_consistency hy)  [5.000]
20.005: (check_correct hy)  [5.000]
