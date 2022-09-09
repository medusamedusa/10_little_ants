

READ_ME


PSO: code adapted from https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/

--------

our_method_test1

10 ants, randomly generated around (0.6, 0.6)
They reach the target with a precision of 0.01.
The target is, in fact, on (0.9, 0.5), and the final positions of ants are (0.89, 0.65/0.66).

--------

PSO_test1

10 particles, randomly generated around (0.6, 0.6)
We considered the target as a position of minimum to be reached.
The particles reach the target with a precision of 0.1, that is, 10 times less precise
than our method. The minimum is on (0.89..., 0.49...), to be approximated as (0.9, 0.5),
and the best solution found by PSO is (0.79..., 0.59,...). 

Objective function: return (x-0.9)**2 + (y-0.5)**2
Particles: X = np.random.rand(2, n_particles)*0.1 + 0.2, V = np.random.rand(2, n_particles)*0.1 + 0.2

-------

PSO_test2

10 particles, randomly generated across the whole arena (1, 1)
We considered the target as a position of minimum to be reached.
--> This configuration does not considering a starting point inside a "nest" as for test1.
The particles reach the target with a precision of 0.01 or 0.0.
The minimum is on (0.89..., 0.49...), to be approximated as (0.9, 0.5),
and the best solution found by PSO is (0.90..., 0.49,...).

Objective function: return (x-0.9)**2 + (y-0.5)**2
Particles: X = np.random.rand(2, n_particles)*0.9, V = np.random.rand(2, n_particles)*0.01

