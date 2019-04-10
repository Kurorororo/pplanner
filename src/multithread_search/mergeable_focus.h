#ifndef MERGEABLE_FOCUS_H_
#define MERGEABLE_FOCUS_H_

#include <memory>
#include <vector>

#include "open_list.h"
#include "open_list_factory.h"

namespace pplanner {

template <typename T>
class MergeableFocus {
 public:
  MergeableFocus() : best_h_(-1), idx_(0) {}

  MergeableFocus(const boost::property_tree::ptree &pt,
                 const std::vector<int> &values, T node, bool is_pref,
                 int plateau_threshold, std::shared_ptr<MergeableFocus> parent)
      : best_h_(values[0]),
        plateau_threshold_(plateau_threshold),
        n_plateau_(0),
        idx_(0),
        size_(0),
        minimum_values_(values),
        priority_(values.size() + 1),
        open_lists_(1, SharedOpenListFactory<T>(pt)),
        parent_(parent) {
    Push(values, node, is_pref);
    UpdatePriority();
  }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  bool IsEmpty() const { return size_ == 0; }

  void Push(const std::vector<int> &values, T node, bool is_pref) {
    ++size_;
    open_lists_[idx_]->Push(values, node, is_pref);
  }

  T Pop();

  const std::vector<int> &Priority() const { return priority_; }

  void Merge(std::shared_ptr<MergeableFocus<T> > focus);

  void Boost() { open_lists_[0]->Boost(); }

  void IncrementNPlateau() { ++n_plateau_; }

  void ClearNPlateau() { n_plateau_ = 0; }

  void UpdatePriority();

 private:
  int best_h_;
  int plateau_threshold_;
  int n_plateau_;
  int idx_;
  int size_;
  std::vector<int> minimum_values_;
  std::vector<int> priority_;
  std::shared_ptr<MergeableFocus> parent_;
  std::vector<std::shared_ptr<OpenList<T> > > open_lists_;
};

template <typename T>
void MergeableFocus<T>::UpdatePriority() {
  priority_[0] = n_plateau_ / plateau_threshold_;

  int arg_min = -1;
  minimum_values_.clear();

  for (int i = 0, n = open_lists_.size(); i < n; ++i) {
    if (open_lists_[i]->IsEmpty()) continue;

    if (arg_min == -1 || open_lists_[i]->MinimumValue() < minimum_values_) {
      arg_min = i;
      minimum_values_ = open_lists_[i]->MinimumValue();
    }
  }

  for (int i = 0, n = minimum_values_.size(); i < n; ++i)
    priority_[i + 1] = minimum_values_[i];
}

template <typename T>
T MergeableFocus<T>::Pop() {
  --size_;
  idx_ = (idx_ + 1) % open_lists_.size();

  if (open_lists_[idx_]->IsEmpty()) {
    for (int i = 0, n = open_lists_.size(); i < n; ++i) {
      idx_ = (idx_ + 1) % open_lists_.size();

      if (!open_lists_[idx_]->IsEmpty()) return open_lists_[idx_]->Pop();
    }
  }

  return open_lists_[idx_]->Pop();
}

template <typename T>
void MergeableFocus<T>::Merge(std::shared_ptr<MergeableFocus<T> > focus) {
  best_h_ = best_h_ < focus->best_h_ ? best_h_ : focus->best_h_;
  size_ += focus->size_;

  if (focus->IsEmpty()) return;

  open_lists_.insert(open_lists_.end(), focus->open_lists_.begin(),
                     focus->open_lists_.end());
}

}  // namespace pplanner

#endif  // MERGEABLE_FOCUS_H_
