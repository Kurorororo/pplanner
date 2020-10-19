# PPlanner
PPlanner is a classical planner implementing parallel heuristic search algorithms.

## Reference

Kuroiwa R, Fukunaga A. 2020. Analyzing and Avoiding Pathological Behavior of Parallel Best-First Search. Proc. the 30th International Conference on Automated Planning and Scheduling. (ICAPS 2020)

## Build

### Docker
The docker container does not support MPI based distributed parallel search methods.

```bash
docker build . -t pplanner
```

### Singularity

```bash
sudo singularity build pplanner.sif Singularity
```

### Manual Build
Please see Dockerfile and Singularity to know the commands to build.

## Examples
Input a SAS+ file and a config file.
The planner generates `sas_plan` after finding a solution.


GBFS using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/GBFS-FF.json
```

HDGBFS using the FF heuristic.

```bash
mpirun -n 16 ./bin/mpi_planner -f output.sas -c config_examples/HDGBFS-FF.json
```

NE using the FF heuristic.

```bash
mpirun -n 16 ./bin/mpi_planner -f output.sas -c config_examples/NE-FF.json
```

LG using the FF heuristic.

```bash
mpirun -n 16 ./bin/mpi_planner -f output.sas -c config_examples/LG-FF.json
```

KPGBFS using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/KPGBFS-FF.json
```

P_{GBFS} using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/PGBFS-FF.json
```

P_{GBFS}/C using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/PGBFS-C-FF.json
```

PUHF using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/PUHF-FF.json
```

SPUHF using the FF heuristic.

```bash
./bin/planner -f output.sas -c config_examples/SPUHF-FF.json
```
