set (CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -DNDEBUG")
set (CMAKE_CXX_FLAGS_DEBUG "-Wall -O0 -pg")

add_library (trie STATIC trie.cc)
