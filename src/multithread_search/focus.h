#ifndef FOCUS_H_
#define FOCUS_H_

#include <memory>
#include <vector>

#include "open_list.h"
#include "open_list_factory.h"

namespace pplanner {

template <typename T>
class Focus {
 public:
  Focus() : best_h_(-1), n_plateau_(0) {}

  Focus(const boost::property_tree::ptree &pt, const std::vector<int> &values,
        T node, bool is_pref)
      : best_h_(values[0]),
        n_plateau_(0),
        priority_(values.size() + 1),
        open_list_(OpenListFactory<std::vector<int>, T>(pt)) {
    Push(values, node, is_pref);
  }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values, T node, bool is_pref) {
    open_list_->Push(values, node, is_pref);
  }

  T Pop() { return open_list_->Pop(); }

  const std::vector<int> &Priority() const { return priority_; }

  void Boost() { open_list_->Boost(); }

  void IncrementNPlateau() { ++n_plateau_; }

  void ClearNPlateau() { n_plateau_ = 0; }

  void UpdatePriority(int plateau_threshold);

 private:
  int best_h_;
  int n_plateau_;
  std::vector<int> priority_;
  std::unique_ptr<OpenList<std::vector<int>, T> > open_list_;
};

template <typename T>
void Focus<T>::UpdatePriority(int plateau_threshold) {
  priority_[0] = n_plateau_ / plateau_threshold;

  for (int i = 0, n = open_list_->MinimumValue().size(); i < n; ++i)
    priority_[i + 1] = open_list_->MinimumValue()[i];
}

}  // namespace pplanner

#endif  // FOCUS_H_
