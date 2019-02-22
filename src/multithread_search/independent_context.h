#ifndef INDEPENDENT_CONTEXT_H_
#define INDEPENDENT_CONTEXT_H_

#include <memory>
#include <vector>

#include "closed_list.h"
#include "open_list.h"
#include "open_list_factory.h"
#include "search_node.h"

namespace pplanner {

template<typename T>
class IndependentContext {
 public:
  IndependentContext() : best_h_(-1), n_plateau_(0) {}

  IndependentContext(const boost::property_tree::ptree &pt,
                     const std::vector<int> &values, T node, bool is_pref)
    : best_h_(values[0]),
      n_plateau_(0),
      priority_(values.size() + 1),
      open_list_(OpenListFactory<T>(pt)),
      closed_list_(std::make_unique<ClosedList>(16)) {
    Push(values, node, is_pref);
  }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values, T node, bool is_pref) {
    open_list_->Push(values, node, is_pref);
  }

  T Pop() { return open_list_->Pop(); }

  const std::vector<int>& Priority() const { return priority_; }

  void Boost() { open_list_->Boost(); }

  void IncrementNPlateau() { ++n_plateau_; }

  void ClearNPlateau() { n_plateau_ = 0; }

  void UpdatePriority(int plateau_threshold);

  bool IsClosed(uint32_t hash, const std::vector<uint32_t> &packed_state)
    const {
    return closed_list_->IsClosed(hash, packed_state);
  }

  std::size_t GetIndex(uint32_t hash,
                       const std::vector<uint32_t> &packed_state) const {
    return closed_list_->GetIndex(hash, packed_state);
  }

  SearchNode* GetItem(std::size_t i) const { return closed_list_->GetItem(i); }

  void Close(std::size_t i, SearchNode *node) { closed_list_->Close(i, node); }

  bool Close(SearchNode* node) { return closed_list_->Close(node); }

 private:
  int best_h_;
  int n_plateau_;
  std::vector<int> priority_;
  std::unique_ptr<OpenList<T> > open_list_;
  std::unique_ptr<ClosedList> closed_list_;
};

template<typename T>
void IndependentContext<T>::UpdatePriority(int plateau_threshold) {
  priority_[0] = n_plateau_ / plateau_threshold;

  for (int i = 0, n = open_list_->MinimumValues().size(); i < n; ++i)
    priority_[i + 1] = open_list_->MinimumValue(i);
}

} // namespace pplanner

#endif // INDEPENDENT_CONTEXT_H_
