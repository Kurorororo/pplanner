#include "search/hdgbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph/distributed_search_graph_with_landmarks.h"

namespace pplanner {

using std::make_shared;
using std::unordered_set;
using std::vector;

void HDGBFS::Init(const boost::property_tree::ptree &pt) {
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (auto use_landmark = pt.get_optional<int>("landmark"))
    graph_ = make_shared<DistributedSearchGraphWithLandmarks>(
        *problem_, closed_exponent, rank_);
  else
    graph_ = make_shared<DistributedSearchGraph>(
        *problem_, closed_exponent, rank_);

  vector<std::shared_ptr<Evaluator> > evaluators;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    evaluators.push_back(EvaluatorFactory(problem_, graph_, e));
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_ = evaluators[0];
      else
        preferring_ = EvaluatorFactory(problem_, graph_, preferring.get());
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);

  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);

  unsigned int buffer_size =
    (graph_->NodeSize() + MPI_BSEND_OVERHEAD) * world_size_ * 50000;
  mpi_buffer_ = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer_, buffer_size);

  outgoing_buffers_.resize(world_size_);
}

int HDGBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;

  while (!ReceiveTermination()) {
    ReceiveNodes();
    if (NoNode()) continue;

    int node = NodeToExpand();
    int goal = Expand(node, state, child, applicable, preferred);

    if (goal != -1) {
      SendTermination();
      return goal;
    }
  }

  return -1;
}

vector<int> HDGBFS::InitialExpand() {
  auto state = problem_->initial();
  int rank = z_hash_->operator()(state) % static_cast<size_t>(world_size_);

  if (rank == rank_) {
    int node = graph_->GenerateNode(state, -1, -1, true, -1);
    ++generated_;
    best_h_ = open_list_->EvaluateAndPush(state, node, true);
    std::cout << "Initial heuristic value: " << best_h_ << std::endl;
    ++evaluated_;
  }

  return state;
}

int HDGBFS::Expand(int node, vector<int> &state, vector<int> &child,
                   vector<int> &applicable, unordered_set<int> &preferred) {
  ++expanded_;

  if (graph_->GetStateAndClosed(node, state) != -1) return -1;
  graph_->Close(node);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
    return -1;
  }

  if (use_preferred_)
    preferring_->Evaluate(state, node, applicable, preferred);

  ++n_preferred_evaluated_;
  n_branching_ += applicable.size();

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);

    bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();
    if (is_preferred) ++n_preferreds_;

    int to_rank = z_hash_->operator()(child) % static_cast<size_t>(world_size_);

    if (to_rank == rank_) {
      std::cout << "local " << node << std::endl;
      int child_node = graph_->GenerateNodeIfNotClosed(
          child, node, o, is_preferred, rank_);
      if (child_node == -1) continue;
      ++generated_;

      Evaluate(child, child_node);
    } else {
      size_t index = outgoing_buffers_[to_rank].size();
      outgoing_buffers_[to_rank].resize(index + graph_->NodeSize());
      unsigned char *buffer = outgoing_buffers_[to_rank].data() + index;
      graph_->BufferNode(node, o, child, buffer);
    }
  }

  SendNodes();

  return -1;
}

void HDGBFS::Evaluate(const vector<int> &state, int node) {
  int h = open_list_->EvaluateAndPush(state, node, false);
  ++evaluated_;

  if (h == -1) {
    ++dead_ends_;
    return;
  }

  if (best_h_ == -1 || h < best_h_) {
    best_h_ = h;

    if (rank_ == 0) {
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }

    if (use_preferred_) open_list_->Boost();
  }
}

void HDGBFS::SendNodes() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i].empty()) continue;
    const unsigned char *d = outgoing_buffers_[i].data();
    MPI_Bsend(d, outgoing_buffers_[i].size(), MPI_BYTE, i, kNodeTag,
              MPI_COMM_WORLD);
    outgoing_buffers_[i].clear();
  }
}

void HDGBFS::ReceiveNodes() {
  int has_received = 0;
  size_t node_size = graph_->NodeSize();
  MPI_Status status;
  MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    incoming_buffer_.resize(d_size);
    MPI_Recv(incoming_buffer_.data(), d_size, MPI_BYTE, source, kNodeTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int n_nodes = d_size / node_size;

    for (int i=0; i<n_nodes; ++i) {
      unsigned char *d = incoming_buffer_.data() + i * node_size;
      int node = graph_->GenerateNodeIfNotClosed(d);

      if (node != -1) {
        ++generated_;
        graph_->State(node, tmp_state_);
        Evaluate(tmp_state_, node);
      }
    }

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received,
               &status);
  }
}

void HDGBFS::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i,kTerminationTag, MPI_COMM_WORLD);
  }
}

bool HDGBFS::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

vector<int> HDGBFS::ExtractPath(int node) {
  vector<int> rec_buffer(world_size_);
  MPI_Gather(
      &node, 1, MPI_INT, rec_buffer.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);
  int goal_process = 0;

  if (rank_ == 0) {
    for (int i=0; i<world_size_; ++i) {
      if (rec_buffer[i] != -1) {
        goal_process = i;
        break;
      }
    }
  }

  MPI_Bcast(&goal_process, 1, MPI_INT, 0, MPI_COMM_WORLD);

  if (goal_process == rank_) {
    //std::cout << "rank " << rank_ << " has goal node" << std::endl;

    int p[2];
    p[0] = graph_->Parent(node);
    p[1] = graph_->Action(node);
    int p_rank = graph_->ParentRank(node);

    //std::cout << "parent rank " << p_rank << std::endl;
    MPI_Bsend(p, 2, MPI_INT, p_rank, kPlanTag, MPI_COMM_WORLD);
  }

  MPI_Barrier(MPI_COMM_WORLD);
  MPI_Status status;

  while (true) {
    int has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kPlanTag, MPI_COMM_WORLD, &has_received, &status);
    int size;

    if (has_received) {
      //std::cout <<  "rank=" << rank_ << std::endl;
      MPI_Get_count(&status, MPI_INT, &size);
      //std::cout <<  "size=" << size << std::endl;
      int source = status.MPI_SOURCE;
      vector<int> plan(size + 1);
      MPI_Recv(plan.data() + 1, size, MPI_INT, source, kPlanTag, MPI_COMM_WORLD,
               MPI_STATUS_IGNORE);
      int current_node = plan[1];
      //std::cout << "node=" << current_node << std::endl;
      int parent = graph_->Parent(current_node);
      //std::cout << "parent " << parent << std::endl;

      if (parent == -1) {
        for (int i=0; i<world_size_; ++i) {
          if (i == rank_) continue;
          MPI_Bsend(NULL, 0, MPI_BYTE, i, kPlanTerminationTag, MPI_COMM_WORLD);
        }

        plan.erase(plan.begin(), plan.begin() + 2);

        return plan;
      }

      plan[0] = parent;
      plan[1] = graph_->Action(current_node);
      int parent_rank = graph_->ParentRank(current_node);
      //std::cout << "parent rank: " << parent_rank << std::endl;

      MPI_Bsend(plan.data(), size + 1, MPI_INT, parent_rank, kPlanTag,
                MPI_COMM_WORLD);
      //std::cout << std::endl;
    }

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kPlanTerminationTag, MPI_COMM_WORLD,
               &has_received, &status);

    if (has_received) {
      int source = status.MPI_SOURCE;
      MPI_Recv(NULL, 0, MPI_BYTE, source, kPlanTerminationTag, MPI_COMM_WORLD,
               MPI_STATUS_IGNORE);

      return vector<int>(0);
    }
  }

  return vector<int>(0);
}

void HDGBFS::Flush() {
  MPI_Status status;
  int has_received = 0;

  MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    incoming_buffer_.resize(d_size);
    MPI_Recv(incoming_buffer_.data(), d_size, MPI_BYTE, source, kNodeTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received,
               &status);
  }
}

void HDGBFS::DumpStatistics() const {
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;
  int n_preferreds = 0;
  int n_preferred_evaluated = 0;
  int n_branching = 0;
  MPI_Allreduce(&expanded_, &expanded, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&evaluated_, &evaluated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&generated_, &generated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&dead_ends_, &dead_ends, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_preferreds_, &n_preferreds, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&n_preferred_evaluated_, &n_preferred_evaluated, 1, MPI_INT,
                MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_branching_, &n_branching, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);

  if (rank_ == 0) {
    std::cout << "Expanded " << expanded << " state(s)" << std::endl;
    std::cout << "Evaluated " << evaluated << " state(s)" << std::endl;
    std::cout << "Generated " << generated << " state(s)" << std::endl;
    std::cout << "Dead ends " << dead_ends << " state(s)" << std::endl;
    std::cout << "Preferred evaluated " << n_preferred_evaluated_ << " state(s)"
              << std::endl;
    std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
              << std::endl;
    double p_p_e = static_cast<double>(n_preferreds)
      / static_cast<double>(n_preferred_evaluated);
    std::cout << "Preferreds per state " << p_p_e << std::endl;
    double b_f = static_cast<double>(n_branching)
      / static_cast<double>(n_preferred_evaluated);
    std::cout << "Average branching factor " << b_f << std::endl;
    double p_p_b = static_cast<double>(n_preferreds)
      / static_cast<double>(n_branching);
    std::cout << "Preferred ratio " << p_p_b  << std::endl;
  }
}

} // namespace pplanner
