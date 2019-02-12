#ifndef NBLOCK_H_
#define NBLOCK_H_

namespace pplanner {

class NBlock {
  NBlock(const boost::property_tree::ptree &open_list_option)
    : is_hot_(false),
      sigma_(0),
      open_list_(OpenListFactory(open_list_option)),
      closed_list_(std::make_unique<ClosedList>(16)) {}

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values, SearchNode* node, bool is_pref) {
    open_list_->Push(values, node, is_pref);
  }

  SearchNode* Pop() { return open_list_->Pop(); }

  bool IsClosed(uint32_t hash, const std::vector<int> &packed_state) const {
    return closed_list_->IsClosed(hash, packed_state);
  }

  bool Close(SearchNode* node) { return closed_list_->Close(node); }

 private:
  bool is_hot_;
  int sigma_;
  std::unique_ptr<OpenList<SearchNode*> > open_list_;
  std::unique_ptr<ClosedList> closed_list_;

};

} // namespace pplanner

#endif // NBLOCK_H_
