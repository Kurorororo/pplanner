#include "search/symmetry_breaking_pddsgbfs.h"

#include <vector>

namespace pplanner {

using std::vector;

int SBPDDSGBFS::Search() {
  auto state = InitialEvaluate(true);

  if (runup() && rank() == initial_rank()) {
    while (n_open_nodes() < world_size() && !NoNode()) {
      int node = Pop();
      int goal = IndependentExpand(node, state, true);

      if (goal != -1) {
        SendTermination();

        return goal;
      }
    }

    Distribute(true);
  }

  while (!ReceiveTermination()) {
    ReceiveNodes();
    RegainNodes();
    if (NoNode()) continue;

    int node = Pop();
    int goal = Expand(node, state, true);

    if (goal != -1 || (limit_expansion() && expanded() > max_expansion())) {
      SendTermination();

      return goal;
    }
  }

  return -1;
}

void SBPDDSGBFS::CallbackOnReceiveNode(int source, const unsigned char *d,
                                       bool no_node) {
  static vector<int> values;
  static vector<int> tmp_state(problem()->n_variables());
  static vector<unsigned char> new_d(node_size(), 0);

  auto g = graph();
  int node = g->GenerateAndCloseNodeFromBytes(d);

  if (node != -1) {
    IncrementGenerated();
    const uint32_t *packed = reinterpret_cast<const uint32_t*>(d + node_size());
    g->Unpack(packed, tmp_state);
    SavePackedState(packed);
    int h = Evaluate(tmp_state, node, values);

    if (h == -1) {
      IncrementDeadEnds();

      return;
    }

    if (no_node
        || (steal_best_ && h < best_h())
        || (steal_better_ && h < MinimumValue(0))) {
      Push(values, node);
    } else {
      size_t h_size = values.size() * sizeof(int);
      unsigned char *b = ExtendOutgoingBuffer(source, node_size() + h_size);
      memcpy(b, values.data(), h_size);
      memcpy(new_d.data(), d, 3 * sizeof(int) + sizeof(uint32_t));
      memcpy(new_d.data() + 3 * sizeof(int) + sizeof(uint32_t),
             packed, g->block_size() * sizeof(uint32_t));
      g->BufferNode(node, new_d.data(), b + h_size);
    }
  }
}

void SBPDDSGBFS::RegainNodes() {
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
      int node = g->GenerateNodeFromBytes(
          IncomingBuffer() + i * unit_size, values);
      const uint32_t *packed = reinterpret_cast<const uint32_t*>(
          IncomingBuffer() + i * unit_size + (n_evaluators() + 3) * sizeof(int)
          + sizeof(uint32_t));
      SavePackedState(packed);
      IncrementGenerated();
      g->SetH(node, values[0]);
      Push(values, node);
    }

    has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kRegainTag, MPI_COMM_WORLD, &has_received, &status);
  }
}

} // namespace pplanner
