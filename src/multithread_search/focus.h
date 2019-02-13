#ifndef FOCUS_H_
#define FOCUS_H_

#include <memory>
#include <vector>

#include "open_list.h"
#include "open_list_factory.h"

namespace pplanner {

template<typename T>
class Focus {
 public:
  Focus() : best_h_(-1), arg_min_(-1) {}

  Focus(const boost::property_tree::ptree &pt, const std::vector<int> &values,
        T node, bool is_pref, int plateau_threshold)
    : best_h_(values[0]),
      arg_min_(0),
      minimum_values_(values),
      open_lists_(1, SharedOpenListFactory<T>(pt)) {
    open_lists_[0]->Push(values, node, is_pref);
  }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  bool IsEmpty() const {
    if (open_lists_.size() == 1)
      return open_lists_[0]->IsEmpty();

    return arg_min_ == -1;
  }

  void Push(const std::vector<int> &values, T node, bool is_pref) {
    open_lists_[0]->Push(values, node, is_pref);

    if (open_lists_.size() > 1 && (IsEmpty() || values < minimum_values_)) {
      minimum_values_ = values;
      arg_min_ = 0;
    }
  }

  T Pop();

  const std::vector<int>& MinimumValues() const {
    if (open_lists_.size() == 1) return open_lists_[0]->MinimumValues();

    return minimum_values_;
  }

  void Merge(std::shared_ptr<Focus<T> > focus);

  void Boost() { open_lists_[0]->Boost(); }

 private:
  int best_h_;
  int arg_min_;
  std::vector<int> minimum_values_;
  std::vector<std::shared_ptr<OpenList<T> > > open_lists_;
};

template<typename T>
T Focus<T>::Pop() {
  auto node = open_lists_[arg_min_]->Pop();

  if (open_lists_.size() == 1) return node;

  if (arg_min_ > 0 && open_lists_[arg_min_]->IsEmpty())
    open_lists_.erase(open_lists_.begin() + arg_min_);

  minimum_values_.clear();
  arg_min_ = -1;

  for (int i = 0, n = open_lists_.size(); i < n; ++i) {
    if (open_lists_[i]->IsEmpty()) continue;

    if (arg_min_ == -1 || open_lists_[i]->MinimumValues() < minimum_values_) {
      minimum_values_ = open_lists_[i]->MinimumValues();
      arg_min_ = i;
    }
  }

  return node;
}

template<typename T>
void Focus<T>::Merge(std::shared_ptr<Focus<T> > focus) {
  best_h_ = best_h_ < focus->best_h_ ? best_h_ : focus->best_h_;

  if (focus->IsEmpty()) return;

  if (IsEmpty() || focus->MinimumValues() < MinimumValues()) {
    arg_min_ = open_lists_.size() + focus->arg_min_;
    minimum_values_ = focus->minimum_values_;
  }

  open_lists_.insert(open_lists_.end(), focus->open_lists_.begin(),
                     focus->open_lists_.end());
}

} // namespace pplanner

#endif // FOCUS_H_
