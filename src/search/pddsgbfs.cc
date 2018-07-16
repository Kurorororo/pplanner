#include "search/pddsgbfs.h"

#include <vector>

namespace pplanner {

using std::vector;

int PDDSGBFS::Search() {
  auto state = InitialEvaluate(true);

  while (!ReceiveTermination()) {
    ReceiveNodes();
    RegainNodes();
    if (NoNode()) continue;

    int node = Pop();
    int goal = Expand(node, state, true);

    if (goal != -1) {
      SendTermination();

      return goal;
    }
  }

  return -1;
}

void PDDSGBFS::CallbackOnReceiveNode(int source, const unsigned char *d,
                                     bool no_node) {
  static vector<int> values;
  static vector<int> tmp_state(problem()->n_variables());

  auto g = graph();
  int node = g->GenerateAndCloseNode(d);

  if (node != -1) {
    IncrementGenerated();
    g->State(node, tmp_state);
    int h = Evaluate(tmp_state, node, values);

    if (h == -1) {
      IncrementDeadEnds();

      return;
    }

    if (no_node || (steal_better_ && h < best_h())) {
      Push(values, node);
    } else {
      size_t h_size = values.size() * sizeof(int);
      unsigned char *b = ExtendOutgoingBuffer(source, node_size() + h_size);
      memcpy(b, values.data(), h_size);
      g->BufferNode(node, d, b + h_size);
    }
  }
}

void PDDSGBFS::RegainNodes() {
  static vector<int> values;

  int has_received = 0;
  MPI_Status status;
  MPI_Iprobe(
      MPI_ANY_SOURCE, kRegainTag, MPI_COMM_WORLD, &has_received, &status);
  size_t unit_size = n_evaluators() * sizeof(int) + node_size();
  auto g = graph();

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    ResizeIncomingBuffer(d_size);
    MPI_Recv(IncomingBuffer(), d_size, MPI_BYTE, source, kRegainTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    size_t n_nodes = d_size / unit_size;

    for (size_t i=0; i<n_nodes; ++i) {
      int node = g->GenerateNode(IncomingBuffer() + i * unit_size, values);
      IncrementGenerated();
      Push(values, node);
    }

    has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kRegainTag, MPI_COMM_WORLD, &has_received, &status);
  }
}

} // namespace pplanner
