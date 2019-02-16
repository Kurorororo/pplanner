#ifndef NBLOCK_H_
#define NBLOCK_H_

#include <atomic>
#include <iostream>
#include <memory>

#include "multithread_search/closed_list.h"
#include "multithread_search/search_node.h"
#include "open_list.h"
#include "open_list_factory.h"

#include <boost/property_tree/ptree.hpp>

namespace pplanner {

class NBlock {
 public:
  NBlock(const boost::property_tree::ptree &open_list_option,
         int abstract_node_id, int closd_exponent=16)
    : abstract_node_id_(abstract_node_id),
      sigma_(0),
      sigma_h_(0),
      heap_idx_(-1),
      minimum_(-1),
      hot_(false),
      inuse_(false),
      open_list_(OpenListFactory<SearchNode*>(open_list_option)),
      closed_list_(std::make_unique<ClosedList>(closd_exponent)) {}

  int abstract_node_id() const { return abstract_node_id_; }

  int sigma() const { return sigma_; }

  void increment_sigma() { ++sigma_; }

  void decrement_sigma() { --sigma_; }

  int sigma_h() const { return sigma_h_; }

  void increment_sigma_h() { ++sigma_h_; }

  void decrement_sigma_h() { --sigma_h_; }

  bool hot() const { return hot_; }

  void set_hot() { hot_ = true; }

  void set_cold() { hot_ = false; }

  bool inuse() const { return inuse_; }

  void use() { inuse_ = true; }

  void unuse() { inuse_ = false; }

  bool is_free() const {
    return sigma_ == 0 && sigma_h_ == 0 && !inuse_ && !open_list_->IsEmpty();
  };

  int heap_idx() const { return heap_idx_; }

  void set_heap_idx(int i) { heap_idx_ = i; }

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values, SearchNode* node, bool is_pref) {
    if (minimum_ == -1 || values[0] < minimum_)
      minimum_ = values[0];

    open_list_->Push(values, node, is_pref);
  }

  SearchNode* Pop() {
    auto top = open_list_->Pop();
    minimum_ = open_list_->IsEmpty() ? -1 : open_list_->MinimumValues()[0];

    return top;
  }

  int priority() const { return minimum_; }

  const std::vector<int>& MinimumValues() const {
    return open_list_->MinimumValues();
  }

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

  void Dump() const {
    std::cout << "id: " << abstract_node_id_
              << " sigma=" << sigma_
              << " sigma_h=" << sigma_h_
              << " hot=" << hot_
              << " inuse=" << inuse_
              << " heap index=" << heap_idx_;

    if (open_list_->IsEmpty())
      std::cout << " empty" << std::endl;
    else
      std::cout << " h=" << MinimumValues()[0] << std::endl;
  }

 private:
  int abstract_node_id_;
  int sigma_;
  int sigma_h_;
  int heap_idx_;
  int minimum_;
  bool hot_;
  bool inuse_;
  std::unique_ptr<OpenList<SearchNode*> > open_list_;
  std::unique_ptr<ClosedList> closed_list_;
};

} // namespace pplanner

#endif // NBLOCK_H_
