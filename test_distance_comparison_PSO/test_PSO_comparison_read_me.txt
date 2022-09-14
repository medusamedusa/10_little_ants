

READ_ME


PSO: code adapted from https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/

--------

Our method (tests a-c, g-i)

10 ants, randomly generated around (0.6, 0.6)
They reach the target with a precision of 0.01.
For tests a-c, the target is in (0.9, 0.5). For tests g-i, the target is in (0.8, 0.9).

--------

PSO Our method (tests d-f, l-n)

10 particles, randomly generated around (0.6, 0.6) (tests d-e, l-m) or scattered (tests f, n)
We considered the target as a position of minimum to be reached.
For tests d-f, the target is in (0.9, 0.5). For tests l-n, the target is in (0.8, 0.9). 

Objective function: (x-0.9)**2 + (y-0.5)**2 (tests d-f); (x-0.8)**2 + (y-0.9)**2 (l-n)


Particles: X = np.random.rand(2, n_particles)*0.1 + 0.2, V = np.random.rand(2, n_particles)*0.1 + 0.2  (tests d-e, l-m); X = np.random.rand(2, n_particles)*0.9, V = np.random.rand(2, n_particles)*0.01 (tests f, n)

