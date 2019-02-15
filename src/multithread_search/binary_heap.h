#ifndef BINARY_HEAP_H_
#define BINARY_HEAP_H_

#include <iostream>
#include <vector>

namespace pplanner {

template<class T>
class BinaryHeap {
 public:
  bool IsEmpty() const { return heap_.empty(); }

  T Top() const { return heap_[0]; }

  void Push(T element) {
    element->set_heap_idx(heap_.size());
    heap_.push_back(element);
    BubbleUp(element->heap_idx());
  }

  T Pop() {
    T x = heap_[0];
    heap_[0] = heap_.back();
    heap_[0]->set_heap_idx(0);
    heap_.pop_back();
    TrickleDown(0);
    x->set_heap_idx(-1);

    return x;
  }

  void Remove(T element) {
    int i = element->heap_idx();

    if (i < 0) return;

    if (i == heap_.size() - 1) {
      element->set_heap_idx(-1);
      heap_.pop_back();

      return;
    }

    Swap(i, heap_.size() - 1);
    heap_.pop_back();
    element->set_heap_idx(-1);

    if (!TrickleDown(i)) BubbleUp(i);
  }

  void Dump() const {
    std::cout << "begin dump" << std::endl;
    std::cout << "size=" << heap_.size() << std::endl;

    for (auto t : heap_)
      t->Dump();

    std::cout << "end dump" << std::endl;
  }

 private:
  int Parent(int i) const { return (i - 1) / 2; }

  int Left(int i) const { return 2 * i + 1; }

  int Right(int i) const { return 2 * i + 2; }

  void Swap(int i, int j) {
    T tmp = heap_[i];
    heap_[i] = heap_[j];
    heap_[i]->set_heap_idx(i);
    heap_[j] = tmp;
    heap_[j]->set_heap_idx(j);
  }

  bool BubbleUp(int i) {
    bool changed = false;
    int parent = (i - 1) / 2;

    while (i >  0 && heap_[i]->priority() < heap_[parent]->priority()) {
      Swap(i, parent);
      i = parent;
      parent = (i - 1) / 2;
      if (!changed) changed = true;
    }

    return changed;
  }

  bool TrickleDown(int i) {
    bool changed = false;
    int n = heap_.size();

    do {
      int j = -1;
      int r = Right(i);
      int l = Left(i);

      if (r < n && heap_[r]->priority() < heap_[i]->priority())
        j = heap_[l]->priority() < heap_[r]->priority() ? l : r;
      else
        j = l < n && heap_[l]->priority() < heap_[i]->priority() ? l : j;

      if (j >= 0) {
        Swap(i, j);
        if (!changed) changed = true;
      }

      i = j;
    } while (i >= 0);

    return changed;
  }

  std::vector<T> heap_;
};

} // namespace pplanner

#endif // BINARY_HEAP_H_
