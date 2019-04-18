#include "search/bmrw.h"

#include <iostream>
#include <unordered_set>

#include "open_list_factory.h"

using std::vector;
using std::size_t;

namespace pplanner {

void BMRW::Init(const boost::property_tree::ptree &pt) {
  if (auto opt = pt.get_optional<int>("n_batch")) n_batch_ = opt.get();

  if (auto opt = pt.get_optional<int>("n_elite")) n_elite_ = opt.get();

  if (auto opt = pt.get_optional<int>("walk_length")) walk_length_ = opt.get();

  int closed_exponent = 22;

  if (auto opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = opt.get();

  graph_ =
      std::make_shared<SearchGraphWithLandmarks>(problem_, closed_exponent);

  lmcount_ =
      std::make_shared<LandmarkCountBase>(problem_, true, false, false, false);

  graph_->InitLandmarks(lmcount_->landmark_graph());

  bool use_preferred = false;
  if (auto opt = pt.get_optional<int>("preferred")) use_preferred = true;

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory<int, int>(open_list_option);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram")) ram = opt.get();

  std::size_t size = graph_->ReserveByRAMSize(ram);
  sequences_.reserve(size);
}

void BMRW::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateAndCloseNode(-1, -1, state);
  ++generated_;

  best_h_ = lmcount_->Evaluate(state, nullptr, graph_->Landmark(node));
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  GenerateChildren(node, best_h_, state);
}

void BMRW::PopStates(Batch &batch) {
  int offset = 0;

  for (int i = 0; i < n_batch_; ++i) {
    int node = -1;
    int h = -1;

    if (open_list_->IsEmpty()) {
      if (offset == 0) offset = i;
      h = batch.hs[(i - offset) % offset];
      node = batch.parents[(i - offset) % offset];
    } else {
      h = open_list_->MinimumValue();
      node = open_list_->Pop();
    }

    batch.hs[i] = h;
    batch.parents[i] = node;
  }
}

void BMRW::RandomWalk(Batch &batch) {
  thread_local std::array<std::vector<int>, 2> state;
  thread_local std::array<std::vector<uint8_t>, 2> landmark;

  for (int id = 0; id < n_batch_; ++id) {
    int node = batch.parents[id];

    graph_->State(node, batch.states[id]);
    state[0] = batch.states[id];
    state[1].resize(state[0].size());
    int index = 0;

    if (graph_->Action(node) != -1) {
      memcpy(batch.landmarks[id].data(), graph_->ParentLandmark(node),
             graph_->n_landmarks_bytes() * sizeof(uint8_t));
      landmark[0] = batch.landmarks[id];
      landmark[1].resize(landmark[0].size());
      std::fill(landmark[1].begin(), landmark[1].end(), 0);

      batch.hs[id] =
          lmcount_->Evaluate(state[0], landmark[0].data(), landmark[1].data());
      ++evaluated_;

      if (batch.hs[id] == -1) continue;

      state[1] = state[0];
      index = 1;
    } else {
      memcpy(batch.landmarks[id].data(), graph_->Landmark(node),
             graph_->n_landmarks_bytes() * sizeof(uint8_t));
      landmark[0] = batch.landmarks[id];
      landmark[1].resize(landmark[0].size());
    }

    int length = 0;
    batch.sequences[id].clear();

    for (int i = 0; i < walk_length_; ++i) {
      int a = generator_->Sample(state[index]);
      ++expanded_;

      if (a == -1) {
        state[0] = batch.states[id];
        landmark[0] = batch.landmarks[id];
        index = 0;
        batch.sequences[id].resize(length);
        ++dead_ends_;
        continue;
      }

      problem_->ApplyEffect(a, state[index], state[1 - index]);
      std::fill(landmark[1 - index].begin(), landmark[1 - index].end(), 0);
      int h = lmcount_->Evaluate(state[1 - index], landmark[index].data(),
                                 landmark[1 - index].data());
      ++evaluated_;

      if (h == -1) {
        state[0] = batch.states[id];
        landmark[0] = batch.landmarks[id];
        index = 0;
        batch.sequences[id].resize(length);
        ++dead_ends_;
        continue;
      }

      batch.sequences[id].push_back(a);

      if (h < batch.hs[id]) {
        batch.hs[id] = h;
        batch.states[id] = state[1 - index];
        batch.landmarks[id] = landmark[1 - index];
        length = batch.sequences[id].size();
      }

      index = 1 - index;
    }

    batch.sequences[id].resize(length);
  }
}

void BMRW::GenerateChildren(int parent, int h, const vector<int> &state) {
  thread_local vector<int> child;
  thread_local vector<int> applicable;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;

    return;
  }

  for (auto o : applicable) {
    problem_->ApplyEffect(o, state, child);
    int node = graph_->GenerateNode(o, parent, state, child);
    ++generated_;
    open_list_->Push(h, node, false);
  }
}

int BMRW::PushStates(const Batch &batch) {
  thread_local std::vector<int> arg_h(n_batch_);

  std::iota(arg_h.begin(), arg_h.end(), 0);

  if (n_elite_ > 0 && n_elite_ < n_batch_) {
    auto cond = [batch](int x, int y) { return batch.hs[x] < batch.hs[y]; };
    std::sort(arg_h.begin(), arg_h.end(), cond);
  }

  int counter = 0;

  for (auto i : arg_h) {
    int h = batch.hs[i];

    if (h == -1) continue;

    int node =
        graph_->GenerateAndCloseNode(-1, batch.parents[i], batch.states[i]);

    if (node == -1) continue;

    ++generated_;
    graph_->SetLandmark(node, batch.landmarks[i].data());
    sequences_.resize(node + 1);
    sequences_[node] = batch.sequences[i];

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
    }

    if (h == 0) return node;

    if (counter < n_elite_) {
      GenerateChildren(node, h, batch.states[i]);
      ++counter;
    } else {
      open_list_->Push(h, node, false);
    }
  }

  return -1;
}

void BMRW::Restart() {
  std::cout << "restart" << std::endl;
  graph_->Clear();
  sequences_.clear();
  InitialEvaluate();
}

int BMRW::Search() {
  InitialEvaluate();
  Batch batch(n_batch_, problem_->n_variables(), graph_->n_landmarks_bytes());

  while (true) {
    if (open_list_->IsEmpty()) Restart();

    PopStates(batch);
    RandomWalk(batch);
    int goal = PushStates(batch);

    if (goal != -1) return goal;
  }
}

std::vector<int> BMRW::ExtractPlan(int node) {
  if (node == -1) return std::vector<int>{-1};

  vector<int> result;

  while (graph_->Parent(node) != -1) {
    if (graph_->Action(node) == -1) {
      result.insert(result.begin(), sequences_[node].begin(),
                    sequences_[node].end());
    } else {
      result.insert(result.begin(), graph_->Action(node));
    }

    node = graph_->Parent(node);
  }

  return result;
}

void BMRW::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

}  // namespace pplanner