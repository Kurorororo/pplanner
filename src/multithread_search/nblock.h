#ifndef NBLOCK_H_
#define NBLOCK_H_

namespace pplanner {

class NBlock {
  NBlock(const boost::property_tree::ptree &open_list_option,
         int abstract_node_id, int closd_exponent=16)
    : abstract_node_id_(abstract_node_id),
      sigma_(0),
      sigma_h_(0),
      hot_(false),
      open_list_(OpenListFactory(open_list_option)),
      closed_list_(std::make_unique<ClosedList>(closd_exponent)) {}

  int abstract_node_id() const { return abstract_node_id_; }

  int sigma() const { return sigma_; }

  void increment_sigma() { ++sigma_; }

  void decrement_sigma() { --sigma_; }

  int sigma_h() const { return sigma_h_; }

  void increment_sigma_h() { ++sigma_h_; }

  void decrement_sigma_h() { --sigma_h_; }

  bool hot() const { return hot_; }

  bool set_hot() { hot_ = true; }

  bool set_cold() { hot_ = false; }

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values, SearchNode* node, bool is_pref) {
    open_list_->Push(values, node, is_pref);
  }

  SearchNode* Pop() { return open_list_->Pop(); }

  const std::vector<int>& MinimumVaues() const {
    return open_list_->MinimumVaues();
  }

  bool IsClosed(uint32_t hash, const std::vector<int> &packed_state) const {
    return closed_list_->IsClosed(hash, packed_state);
  }

  bool Close(SearchNode* node) { return closed_list_->Close(node); }

 private:
  int abstract_node_id_;
  int sigma_;
  int sigma_h_;
  bool hot_;
  std::unique_ptr<OpenList<SearchNode*> > open_list_;
  std::unique_ptr<ClosedList> closed_list_;
};

} // namespace pplanner

#endif // NBLOCK_H_
